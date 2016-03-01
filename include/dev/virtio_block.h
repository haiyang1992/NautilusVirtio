#ifndef __VIRTIO_BLOCK
#define __VIRTIO_BLOCK

struct virtio_block_request{
  uint32_t type; //0: Read, 1: Write
  uint32_t priority; //0:Low
  uint64_t sector;
  unsigned char data[255];
  uint8_t status; // 0:OK 1:Error 2:Unsupported
};

#endif
