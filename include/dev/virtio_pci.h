#ifndef __VIRTIO_PCI
#define __VIRTIO_PCI
#include "dev/virtio_ring.h"

#define MAX_VRINGS 4

enum virtio_pci_dev_type { VIRTIO_PCI_NET, VIRTIO_PCI_BLOCK, VIRTIO_PCI_OTHER };

/*struct vring {
  // The actual descriptors (16 bytes each 
  struct vring_desc desc[qsz];

  // A ring of available descriptor heads with free-running index.
  struct vring_avail avail;

  // Padding to the next 4096 boundary.
  char pad[];

  // A ring of used descriptor heads with free-running index.
  struct vring_used used;
};*/

struct virtio_pci_vring {
  uint64_t size_bytes;

  // data is in charge of memory allcation/deallocation of the vring
  uint8_t *data ;
  // the start of the vring from our and the device's perspective
  uint8_t *aligned_data;

  struct virtq vq;
};

struct virtio_pci_dev {
  enum virtio_pci_dev_type type;
  char name[32];

  // for our linked list of virtio devices
  struct list_head virtio_node;

  struct pci_dev *pci_dev;

  uint8_t   pci_intr;  // number on bus
  uint8_t   intr_vec;  // number we will see

  // Where registers are mapped into the I/O address space
  uint16_t  ioport_start;
  uint16_t  ioport_end;  

  // Where registers are mapped into the physical memory address space
  uint64_t  mem_start;
  uint64_t  mem_end;

  // The number of vrings in use
  uint8_t num_vrings;
  struct virtio_pci_vring vring[MAX_VRINGS];
};

int virtio_pci_init(struct naut_info * naut);
int virtio_pci_deinit();


#endif
