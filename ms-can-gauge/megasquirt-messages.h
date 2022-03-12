/* These structures are used to hold the header and data from the messages
 * coming from the megasquirt over the CANbus. */

// Pack/Unpack the Megasquirt extended message format header
typedef struct msg_packed_int
{
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
} msg_packed_int;

typedef struct msg_bit_info
{
  unsigned int spare:2;
  unsigned int block_h:1;
  unsigned int block:4;
  unsigned int to_id:4;
  unsigned int from_id:4;
  unsigned int msg_type:3;
  unsigned int offset:11;
} msg_bit_info;

typedef union
{
  unsigned int i;
  msg_packed_int b;
  msg_bit_info values;
} msg_packed;

// Unpack the vars from the payload of a MSG_REQ packet
typedef struct msg_req_data_packed_int
{
  unsigned char b2;
  unsigned char b1;
  unsigned char b0;
} msg_req_data_packed_int;

typedef struct msq_req_data_bit_info
{
  unsigned int varbyt:4;
  unsigned int spare:1;
  unsigned int varoffset:11;
  unsigned int varblk:4;
} msg_req_data_bit_info;

typedef union
{
  msg_req_data_packed_int bytes;
  msg_req_data_bit_info values;
} msg_req_data_raw;