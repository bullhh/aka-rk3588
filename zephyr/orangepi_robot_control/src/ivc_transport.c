/* SPDX-License-Identifier: Apache-2.0 */

#include "ivc_transport.h"

#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#define AXVISOR_HVC_SUBSCRIBE 4U
#define IVC_PUBLISHER_VM_ID 1U
#define IVC_CHANNEL_KEY 0x49564301U

#define IVC_REGION_MAGIC 0x49564332U
#define IVC_REGION_VERSION 2U
#define IVC_MESSAGE_REQUEST 1U
#define IVC_RING_CAPACITY 16U
#define IVC_SLOT_PAYLOAD_SIZE 48U
#define IVC_SUBSCRIBE_RETRY_MS 100U

struct ivc_region_header {
	uint32_t magic;
	uint32_t version;
	uint32_t header_size;
	uint32_t region_size;
	uint32_t features;
	uint32_t publisher_to_subscriber_offset;
	uint32_t subscriber_to_publisher_offset;
	uint32_t ring_size;
};

struct ivc_message_slot {
	uint64_t sequence;
	uint32_t length;
	uint32_t kind;
	uint8_t payload[IVC_SLOT_PAYLOAD_SIZE];
} __aligned(64);

struct ivc_ring {
	uint32_t direction;
	uint32_t capacity;
	uint32_t slot_payload_size;
	uint32_t head;
	uint32_t tail;
	uint32_t reserved[3];
	struct ivc_message_slot slots[IVC_RING_CAPACITY];
} __aligned(64);

struct ivc_region {
	uint64_t publisher_id;
	uint64_t key;
	struct ivc_region_header header;
	struct ivc_ring publisher_to_subscriber;
	struct ivc_ring subscriber_to_publisher;
} __aligned(64);

static volatile uintptr_t subscribe_base_gpa;
static volatile uintptr_t subscribe_size;

BUILD_ASSERT(sizeof(struct ivc_message_slot) == 64);
BUILD_ASSERT(sizeof(struct ivc_ring) == 1088);
BUILD_ASSERT(offsetof(struct ivc_region, publisher_to_subscriber) == 64);
BUILD_ASSERT(offsetof(struct ivc_region, subscriber_to_publisher) == 1152);
BUILD_ASSERT(sizeof(struct ivc_region) == 2240);

static long axvisor_hvc(uint64_t code, uint64_t arg0, uint64_t arg1,
			 uint64_t arg2, uint64_t arg3)
{
	register uint64_t x0 __asm__("x0") = code;
	register uint64_t x1 __asm__("x1") = arg0;
	register uint64_t x2 __asm__("x2") = arg1;
	register uint64_t x3 __asm__("x3") = arg2;
	register uint64_t x4 __asm__("x4") = arg3;

	__asm__ volatile("hvc #0\n\tnop"
			 : "+r"(x0)
			 : "r"(x1), "r"(x2), "r"(x3), "r"(x4)
			 : "memory");
	return (long)x0;
}

static bool protocol_ready(const struct ivc_region *region, size_t mapped_size)
{
	const uint32_t magic = __atomic_load_n(&region->header.magic, __ATOMIC_ACQUIRE);
	const uint32_t version = __atomic_load_n(&region->header.version, __ATOMIC_ACQUIRE);
	const uint32_t region_size =
		__atomic_load_n(&region->header.region_size, __ATOMIC_ACQUIRE);

	return region->publisher_id == IVC_PUBLISHER_VM_ID &&
	       region->key == IVC_CHANNEL_KEY && magic == IVC_REGION_MAGIC &&
	       version == IVC_REGION_VERSION && region_size >= sizeof(*region) &&
	       mapped_size >= sizeof(*region);
}

int robot_ivc_subscribe(struct robot_ivc *ivc)
{
	uint64_t attempts = 0;

	if (ivc == NULL) {
		return -EINVAL;
	}

	for (;;) {
		subscribe_base_gpa = 0;
		subscribe_size = 0;
		const uintptr_t base_slot_gpa =
			k_mem_phys_addr((void *)&subscribe_base_gpa);
		const uintptr_t size_slot_gpa = k_mem_phys_addr((void *)&subscribe_size);
		const long status = axvisor_hvc(AXVISOR_HVC_SUBSCRIBE,
						 IVC_PUBLISHER_VM_ID,
						 IVC_CHANNEL_KEY,
						 base_slot_gpa,
						 size_slot_gpa);
		const uintptr_t base_gpa = subscribe_base_gpa;
		const uintptr_t size = subscribe_size;

		++attempts;
		if (status == 0) {
			mm_reg_t mapped = 0;

			/* device_map() adds K_MEM_DIRECT_MAP on this ARM64 build.  The
			 * shared GPA must be identity-mapped because AxVisor installs the
			 * subscriber's stage-2 mapping at the GPA returned by the HVC.
			 * Keep it non-cacheable to match the current ArceOS axivc mapping
			 * and avoid cross-VM cache aliasing.
			 */
			device_map(&mapped, base_gpa, size, K_MEM_CACHE_NONE);
			ivc->region = (void *)mapped;
			ivc->size = size;

			const struct ivc_region *region = (const struct ivc_region *)mapped;
			while (!protocol_ready(region, size)) {
				k_msleep(IVC_SUBSCRIBE_RETRY_MS);
			}
			printk("ZEPHYR_IVC_READY publisher=%u key=0x%x base=0x%lx "
			       "size=%zu attempts=%llu\n",
			       IVC_PUBLISHER_VM_ID, IVC_CHANNEL_KEY,
			       (unsigned long)base_gpa, size, attempts);
			return 0;
		}

		if (attempts == 1U || attempts % 30U == 0U) {
			printk("ZEPHYR_IVC_WAIT attempts=%llu status=%ld\n", attempts,
			       status);
		}
		k_msleep(IVC_SUBSCRIBE_RETRY_MS);
	}
}

int robot_ivc_try_receive(struct robot_ivc *ivc, void *payload, size_t capacity,
			  size_t *length, uint64_t *sequence)
{
	if (ivc == NULL || ivc->region == NULL || payload == NULL || length == NULL ||
	    sequence == NULL) {
		return -EINVAL;
	}

	struct ivc_region *region = ivc->region;
	struct ivc_ring *ring = &region->publisher_to_subscriber;
	const uint32_t head = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
	const uint32_t tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);

	if (head == tail) {
		return -EAGAIN;
	}

	struct ivc_message_slot *slot = &ring->slots[head % IVC_RING_CAPACITY];
	const uint32_t slot_length = __atomic_load_n(&slot->length, __ATOMIC_RELAXED);
	const uint32_t kind = __atomic_load_n(&slot->kind, __ATOMIC_RELAXED);

	if (kind != IVC_MESSAGE_REQUEST || slot_length > IVC_SLOT_PAYLOAD_SIZE ||
	    slot_length > capacity) {
		/* Drop one malformed slot so a corrupt producer cannot permanently
		 * wedge the safety controller on the same ring head.
		 */
		__atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
		return -EPROTO;
	}

	for (uint32_t index = 0; index < slot_length; ++index) {
		((uint8_t *)payload)[index] = slot->payload[index];
	}
	*sequence = __atomic_load_n(&slot->sequence, __ATOMIC_RELAXED);
	*length = slot_length;
	__atomic_store_n(&ring->head, head + 1U, __ATOMIC_RELEASE);
	return 0;
}
