#ifndef __VIRTIO_BLOCK
#define __VIRTIO_BLOCK
#include <dev/virtio_pci.h>
#include <nautilus/list.h>

/* Feature bits */
#define VIRTIO_BLK_F_BARRIER    0	/* Device supports request barriers */ 
#define VIRTIO_BLK_F_SIZE_MAX   1	/* Maximum size of any single segment */
#define VIRTIO_BLK_F_SEG_MAX    2	/* Maximum number of segments in a request */
#define VIRTIO_BLK_F_GEOMETRY   4	/* Disk style geometry */
#define VIRTIO_BLK_F_RO         5	/* Device is read-only */
#define VIRTIO_BLK_F_BLK_SIZE   6	/* Block size of disk */
#define VIRTIO_BLK_F_SCSI       7	/* Supports scsi commands */
#define VIRTIO_BLK_F_FLUSH      9	/* Cache flush command support */
#define VIRTIO_BLK_F_TOPOLOGY   10	/* Topology information is available */
#define VIRTIO_BLK_F_CONFIG_WCE 11	/* Device can toggle between writeback and writethrough methods */

struct virtio_block_request{
  /* read */
  #define VIRTIO_BLK_T_IN	0
  /* write */
  #define VIRTIO_BLK_T_OUT	1
  /* flush */
  #define VIRTIO_BLK_T_FLUSH	4
  uint32_t type;
  uint32_t priority;
  uint64_t sector;
  unsigned char data[255];
  
  /* success */
  #define VIRTIO_BLK_S_OK	0
  /* device or driver error */
  #define VIRTIO_BLK_S_IOERR	1
  /* request unsupported by device */
  #define VIRTIO_BLK_S_UNSUPP	2
  uint8_t status;
};

// this is the abstract base class for block devices
// it's  all the functions that a specific block device
// presents
struct block_dev_int {
    // these are function pointers
    uint64_t (*get_block_size)(void *state);
    uint64_t (*get_num_blocks)(void *state);
    int (*read_block)(void *state, uint64_t blocknum, uint8_t *dest);
    int (*write_block)(void *state, uint64_t blocknum, uint8_t *src);

};

// this contains the data part of a block device as far as the 
// block device layer knows about it. 
struct block_dev {
   // has some name
   char name[32];
   // will be in a list of all block devices in the system
   struct hlist_node block_dev_list_node;
   // a pointer to state supplied by the driver
   void *state;
   // the actual driver's interface functions
   struct block_dev_int interface;
};

int write_block(void*, uint64_t, uint8_t*);

int read_block(void*, uint64_t, uint8_t*);

int blockrq_enqueue(struct virtio_pci_dev*, struct virtio_block_request*, uint8_t);

struct block_dev * block_dev_register(char*, struct block_dev_int*, void*);

int block_dev_unregister(struct block_dev*);

#endif
