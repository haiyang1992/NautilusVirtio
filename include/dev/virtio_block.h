#ifndef __VIRTIO_BLOCK
#define __VIRTIO_BLOCK
#include <dev/virtio_pci.h>


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

static int blockrq_enqueue(struct virtio_pci_dev *dev, struct virtio_block_request blkrq[], uint8_t size){  
  int enqueue_status;
  uint16_t i;

  for (i=0;i<size;i++){
    uint8_t head = 0;
    uint16_t flags = 1;
    if (i == 0) 
      head = 1;
    if (i == size - 1) 
      flags = 2;
    enqueue_status = virtio_enque_request(dev, 0, (uint64_t)&blkrq[i], (uint32_t)(sizeof(blkrq[i])), flags, head);
    if (enqueue_status){
      ERROR("Enqueue of block number %d failed\n", i + 1);
      return -1;
    }
  }
  DEBUG("Enqueued block request\n");
  return 0;
};

#endif
