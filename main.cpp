#include <iostream>
#include "IPv4Layer.h"
#include "Packet.h"
#include "PcapFileDevice.h"
#include "UdpLayer.h"
#include "sms_types.h"
#include "checksum.h"

SMS_TRANSPORT_DATA_FRAME_T s_dataFrame;
SMS_PORT_HEADER_T s_portHeader;
SMS_INSTRUCTION_HEADER_T s_instructionHeader;
SMS_INSTRUCTION_T as_instruction[NOF_INSTRUCTIONS];
SMS_TARGET_HEADER_T s_targetHeader;
SMS_TARGET_T as_target[NOF_TARGETS];
SMS_OBJECT_HEADER_T s_objectHeader;
SMS_OBJECT_T as_object[NOF_OBJECTS];
SMS_DYNAMICS_DATA_T s_comDynamicsData;
SMS_OCC_GRID_OUTPUT_HEADER_T s_occGridOutHeader;
SMS_OCC_CELL_T as_occCell[NOF_OCC_CELLS];

bool crc_tabccitt_init_ = false;
uint16_t crc_tabccitt_[256];

int pktCount = 0;

void init_crcccitt_tab() {
  uint16_t crc;
  uint16_t c;

  for (uint16_t i = 0; i < 256; ++i)
  {
    crc = 0;
    c = i << 8;

    for (uint16_t j = 0; j < 8; ++j)
    {
      if ((crc ^ c) & 0x8000)
        crc = (crc << 1) ^ CRC_POLY_CCITT;
      else
        crc = crc << 1;

      c = c << 1;
    }

    crc_tabccitt_[i] = crc;
  }

  crc_tabccitt_init_ = true;
}

uint16_t crc16_ccitt_generic(const unsigned char *input_str, size_t num_bytes, uint16_t start_value) {
  const unsigned char *ptr = input_str;
  uint16_t crc = start_value;

  if (!crc_tabccitt_init_)
    init_crcccitt_tab();

  if (ptr != NULL)
  {
    uint16_t short_c;
    uint16_t tmp;

    for (size_t a = 0; a < num_bytes; a++)
    {
      short_c = 0x00ff & (unsigned short)*ptr;
      tmp = (crc >> 8) ^ short_c;
      crc = (crc << 8) ^ crc_tabccitt_[tmp];
      ptr++;
    }
  }

  return crc;
}

float convert_hex_to_float(uint8_t *data) {
  uint32_t u = 0;
  u = data[0] << 24;
  u |= data[1] << 16;
  u |= data[2] << 8;
  u |= data[3];

  return *reinterpret_cast<float *>(&u);
}

void send_data_proc(int count) {
  for (int i = 0; i < count; i++) {
    std::cout << "X : " << as_object[i].f_xPos << std::endl;
    std::cout << "Y : " << as_object[i].f_yPos << std::endl;
    std::cout << "S : " << as_object[i].f_speedAbs * 3600.0f / 1000.0f << std::endl;
    std::cout << "L : " << as_object[i].f_length << std::endl;
    std::cout << "I : " << as_object[i].i16_trkChannel << std::endl;
    std::cout << "Q : " << as_object[i].f_quality << std::endl;
    if (as_object[i].f_quality >= 0.2f)
      pktCount++;
  }
}

void parse_SMSObject_V2(uint8_t *u8_input_buffer, int index) {
  // Decode object data
  memset(&as_object[index], 0, sizeof(SMS_OBJECT_T));

  //as_object[index].f_xPos = (float)((u8_input_buffer[0] << 24) | (u8_input_buffer[1] << 16) | (u8_input_buffer[2] << 8) | u8_input_buffer[3]);
  as_object[index].f_xPos = convert_hex_to_float(u8_input_buffer);
  //as_object[index].f_yPos = (float)((u8_input_buffer[4] << 24) | (u8_input_buffer[5] << 16) | (u8_input_buffer[6] << 8) | u8_input_buffer[7]);
  as_object[index].f_yPos = convert_hex_to_float(&u8_input_buffer[4]);
  //as_object[index].f_speedAbs = (float)((u8_input_buffer[8] << 24) | (u8_input_buffer[9] << 16) | (u8_input_buffer[10] << 8) | u8_input_buffer[11]);
  as_object[index].f_speedAbs = convert_hex_to_float(&u8_input_buffer[8]);
  //as_object[index].f_heading = (float)((u8_input_buffer[12] << 24) | (u8_input_buffer[13] << 16) | (u8_input_buffer[14] << 8) | u8_input_buffer[15]);
  as_object[index].f_heading = convert_hex_to_float(&u8_input_buffer[12]);
  //as_object[index].f_length = (float)((u8_input_buffer[16] << 24) | (u8_input_buffer[17] << 16) | (u8_input_buffer[18] << 8) | u8_input_buffer[19]);
  as_object[index].f_length = convert_hex_to_float(&u8_input_buffer[16]);
  //as_object[index].f_quality = (float)((u8_input_buffer[24] << 24) | (u8_input_buffer[25] << 16) | (u8_input_buffer[26] << 8) | u8_input_buffer[27]);
  as_object[index].f_quality = convert_hex_to_float(&u8_input_buffer[24]);
  //as_object[index].f_accel = (float)((u8_input_buffer[28] << 24) | (u8_input_buffer[29] << 16) | (u8_input_buffer[30] << 8) | u8_input_buffer[31]);
  as_object[index].f_accel = convert_hex_to_float(&u8_input_buffer[28]);
  as_object[index].i16_trkChannel = (int16_t)((u8_input_buffer[32] << 8) | u8_input_buffer[33]);
  as_object[index].u16_idleCycles = (u8_input_buffer[34] << 8) | u8_input_buffer[35];
  as_object[index].u8_statusFlags = u8_input_buffer[39];
}

void parse_SMSObject_V3(uint8_t *u8_input_buffer, int index) {
  // Decode object data
  memset(&as_object[index], 0, sizeof(SMS_OBJECT_T));

  //as_object[index].f_xPos = (float)((u8_input_buffer[0] << 24) | (u8_input_buffer[1] << 16) | (u8_input_buffer[2] << 8) | u8_input_buffer[3]);
  as_object[index].f_xPos = convert_hex_to_float(u8_input_buffer);
  //as_object[index].f_yPos = (float)((u8_input_buffer[4] << 24) | (u8_input_buffer[5] << 16) | (u8_input_buffer[6] << 8) | u8_input_buffer[7]);
  as_object[index].f_yPos = convert_hex_to_float(&u8_input_buffer[4]);
  //as_object[index].f_zPos = (float)((u8_input_buffer[8] << 24) | (u8_input_buffer[9] << 16) | (u8_input_buffer[10] << 8) | u8_input_buffer[11]);
  as_object[index].f_zPos = convert_hex_to_float(&u8_input_buffer[8]);
  //as_object[index].f_speedAbs = (float)((u8_input_buffer[12] << 24) | (u8_input_buffer[13] << 16) | (u8_input_buffer[14] << 8) | u8_input_buffer[15]);
  as_object[index].f_speedAbs = convert_hex_to_float(&u8_input_buffer[12]);
  //as_object[index].f_heading = (float)((u8_input_buffer[16] << 24) | (u8_input_buffer[17] << 16) | (u8_input_buffer[18] << 8) | u8_input_buffer[19]);
  as_object[index].f_heading = convert_hex_to_float(&u8_input_buffer[16]);
  //as_object[index].f_length = (float)((u8_input_buffer[20] << 24) | (u8_input_buffer[21] << 16) | (u8_input_buffer[22] << 8) | u8_input_buffer[23]);
  as_object[index].f_length = convert_hex_to_float(&u8_input_buffer[20]);
  //as_object[index].f_quality = (float)((u8_input_buffer[28] << 24) | (u8_input_buffer[29] << 16) | (u8_input_buffer[30] << 8) | u8_input_buffer[31]);
  as_object[index].f_quality = convert_hex_to_float(&u8_input_buffer[28]);
  //as_object[index].f_accel = (float)((u8_input_buffer[32] << 24) | (u8_input_buffer[33] << 16) | (u8_input_buffer[34] << 8) | u8_input_buffer[35]);
  as_object[index].f_accel = convert_hex_to_float(&u8_input_buffer[32]);
  as_object[index].i16_trkChannel = (int16_t)((u8_input_buffer[36] << 8) | u8_input_buffer[37]);
  as_object[index].u16_idleCycles = (u8_input_buffer[38] << 8) | u8_input_buffer[39];
  as_object[index].u8_statusFlags = u8_input_buffer[43];
}

void parse_inputSMSObjects(uint8_t *u8_input_buffer, int i32_len, int16_t i16_portMajVer) {
  int objHeaderLen = sizeof(s_objectHeader);

  if (objHeaderLen <= i32_len)
  {
    // Decode object header
    memset(&s_objectHeader, 0, objHeaderLen);

    //s_objectHeader.f_cycleTime = (float)((u8_input_buffer[0] << 24) | (u8_input_buffer[1] << 16) | (u8_input_buffer[2] << 8) | u8_input_buffer[3]);
    s_objectHeader.f_cycleTime = convert_hex_to_float(u8_input_buffer);
    s_objectHeader.u16_nOfObjects = (u8_input_buffer[4] << 8) | u8_input_buffer[5];

    // Print object header information
    //printSMSObjectHeader();

    // Decode objects
    uint8_t u8_objBody[MAX_PAYLOAD_SIZE];
    memcpy(&u8_objBody, u8_input_buffer + objHeaderLen, i32_len - objHeaderLen);

    int objBodyLen = 0;

    if (i16_portMajVer > 2)
    {
      // zPos is included in the Object from the port header major version v3
      objBodyLen = s_objectHeader.u16_nOfObjects * 44; //sizeof(SMS_OBJECT_T);
      if (objBodyLen <= (i32_len - objHeaderLen))
      {
        for (int i = 0; i < s_objectHeader.u16_nOfObjects; ++i)
        {
          parse_SMSObject_V3(/*&*/ u8_objBody + (i * 44), i);
          // Print object information
          //printSMSObject_V3(i);
        }
        send_data_proc(s_objectHeader.u16_nOfObjects);
      }
      else
      {
        //printf(" Insufficient payload to decode objects \n");
      }
    }
    else
    {
      // zPos is not included in the Object prior to the port header major version v3
      objBodyLen = s_objectHeader.u16_nOfObjects * (sizeof(SMS_OBJECT_T) - 4);
      if (objBodyLen <= (i32_len - objHeaderLen))
      {
        for (int i = 0; i < s_objectHeader.u16_nOfObjects; ++i)
        {
          parse_SMSObject_V2(/*&*/ u8_objBody, i);
          // Print object information
          //printSMSObject_V2(i);
        }
        send_data_proc(s_objectHeader.u16_nOfObjects);
      }
      else
      {
        //printf(" Insufficient payload to decode objects \n");
      }
    }
  }
  else
  {
    //printf(" Insufficient payload to decode objects data \n");
  }
}

void parse_inputSMSTransportPayload(uint8_t *u8_input_buffer, int i32_len) {
  int portHeaderLen = sizeof(s_portHeader); // 24 bytes

  if (portHeaderLen <= i32_len)
  {
    // Decode port header
    memset(&s_portHeader, 0, sizeof(s_portHeader));

    s_portHeader.u32_portIdentifier |= u8_input_buffer[0] << 24;
    s_portHeader.u32_portIdentifier |= u8_input_buffer[1] << 16;
    s_portHeader.u32_portIdentifier |= u8_input_buffer[2] << 8;
    s_portHeader.u32_portIdentifier |= u8_input_buffer[3];

    s_portHeader.i16_portMajVer = (int16_t)((u8_input_buffer[4] << 8) | u8_input_buffer[5]);
    s_portHeader.i16_portMinVer = (int16_t)((u8_input_buffer[6] << 8) | u8_input_buffer[7]);

    s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[8] << 56;
    s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[9] << 48;
    s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[10] << 40;
    s_portHeader.u64_portTimeStamp |= (uint64_t)u8_input_buffer[11] << 32;
    s_portHeader.u64_portTimeStamp |= u8_input_buffer[12] << 24;
    s_portHeader.u64_portTimeStamp |= u8_input_buffer[13] << 16;
    s_portHeader.u64_portTimeStamp |= u8_input_buffer[14] << 8;
    s_portHeader.u64_portTimeStamp |= u8_input_buffer[15];

    s_portHeader.u32_portSize |= u8_input_buffer[16] << 24;
    s_portHeader.u32_portSize |= u8_input_buffer[17] << 16;
    s_portHeader.u32_portSize |= u8_input_buffer[18] << 8;
    s_portHeader.u32_portSize |= u8_input_buffer[19];

    s_portHeader.u8_portEndianess = u8_input_buffer[20];

    s_portHeader.u8_portIndex = u8_input_buffer[21];

    s_portHeader.u8_portHeaderMajVer = u8_input_buffer[22];

    s_portHeader.u8_portHeaderMinVer = u8_input_buffer[23];

    // Print sms port header information
    //printSMSPortHeader();

    // Decode port body
    uint8_t u8_portBody[MAX_PAYLOAD_SIZE];
    memcpy(u8_portBody, u8_input_buffer + portHeaderLen, i32_len - portHeaderLen);

    switch (s_portHeader.u32_portIdentifier)
    {
      case INSTRUCTIONS:
        // Decode instructions
        if (sizeof(s_instructionHeader) <= (i32_len - portHeaderLen))
        {
          //parse_inputSMSInstructions(/*&*/ u8_portBody, i32_len - portHeaderLen);
        }
        else
        {
          //printf(" Insufficient payload to decode instruction data \n");
        }
        break;
      case TARGET_LIST:
        // Decode targets
        if (sizeof(s_targetHeader) <= (i32_len - portHeaderLen))
        {
          //parse_inputSMSTargets(/*&*/ u8_portBody, i32_len - portHeaderLen);
        }
        else
        {
          //printf(" Insufficient payload to decode target data \n");
        }
        break;
      case OBJECT_LIST:
        // Decode objects
        if (sizeof(s_objectHeader) <= (i32_len - portHeaderLen))
        {
          parse_inputSMSObjects(/*&*/ u8_portBody, i32_len - portHeaderLen, s_portHeader.i16_portMajVer);
        }
        else
        {
          //printf(" Insufficient payload to decode object data \n");
        }
        break;
      case COM_DYNAMICS:
        // Decode com dynamics data
        if (sizeof(s_comDynamicsData) <= (i32_len - portHeaderLen))
        {
          //parse_inputSMSComDynamicsData(/*&*/ u8_portBody, i32_len - portHeaderLen);
        }
        else
        {
          //printf(" Insufficient payload to decode com dynamics data \n");
        }
        break;
      case OCC_GRID_OUTPUT:
        // Decode occupancy grid output list
        if (sizeof(s_occGridOutHeader) <= (i32_len - portHeaderLen))
        {
          //parse_inputSMSOccGridOutput(/*&*/ u8_portBody, i32_len - portHeaderLen);
        }
        else
        {
          //printf(" Insufficient payload to decode occupancy grid output list \n");
        }
        break;
      default:
        //printf("Unknown port identifier[%u]\n", s_portHeader.u32_portIdentifier);
        break;
    }
  }
  else
  {
    //printf(" Insufficient payload to decode port data \n");
  }
}

void parse_inputSMSTransportProtocol(uint8_t *u8_input_buffer, uint32_t u32_len) {
  state_t s_state_Header = START_PATTERN;
  uint8_t u8_input = 0;

  for (uint16_t u16_i = 0; u16_i < u32_len; ++u16_i) {
    u8_input = u8_input_buffer[u16_i];

    switch (s_state_Header) {
      case START_PATTERN:
        // Looking for the Startpattern 0x7E
        if (u8_input == 0x7E) {
          // reset or initialze the data frame structure at the start
          if (!s_dataFrame.b_hasNext) {
            memset(&s_dataFrame, 0, sizeof(s_dataFrame));
          }

          s_dataFrame.u8_startPattern = u8_input;
          s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

          s_state_Header = PROTOCOL_VERSION;
        }
        break;
      case PROTOCOL_VERSION:
        // Protocol Version is 1
        if (u8_input == 0x1) {
          s_dataFrame.u8_protocolVersion = u8_input;
          s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
          s_state_Header = HEADER_LENGTH;
        } else {
          s_state_Header = START_PATTERN;
        }
        break;
      case HEADER_LENGTH:
        // store the Headerlength
        s_dataFrame.u8_headerLength = u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = PAYLOAD_LENGTH_H;
        break;
      case PAYLOAD_LENGTH_H:
        // store payload length high byte
        s_dataFrame.u16_payloadLength |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = PAYLOAD_LENGTH_L;
        break;
      case PAYLOAD_LENGTH_L:
        // store payload length low byte
        s_dataFrame.u16_payloadLength |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = APPPROTOCOL_TYPE;
        break;
      case APPPROTOCOL_TYPE:
        // App Protocol type 8 = PORT Application Protocol
        if (u8_input == 8) {
          s_dataFrame.u8_AppProtocolType = u8_input;
          s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
          s_state_Header = FLAG_1;
        } else {
          s_state_Header = START_PATTERN;
        }
        break;
      case FLAG_1:
        // store Flag
        s_dataFrame.u32_flags |= u8_input << 24;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = FLAG_2;
        break;
      case FLAG_2:
        // store Flag
        s_dataFrame.u32_flags |= u8_input << 16;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = FLAG_3;
        break;
      case FLAG_3:
        // store Flag
        s_dataFrame.u32_flags |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = FLAG_4;
        break;
      case FLAG_4:
        // store Flag
        s_dataFrame.u32_flags |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

        if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_MSG_COUNTER) == SMS_PROTOCOL_FLAG_MSG_COUNTER) &&
            (s_dataFrame.u8_headerLength > 12)) {
          s_state_Header = MSG_COUNTER_1;
        } else {
          s_state_Header = HEADER_CRC16_H;
        }
        break;
      case MSG_COUNTER_1:
        s_dataFrame.u16_msgCounter |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = MSG_COUNTER_2;
        break;
      case MSG_COUNTER_2:
        s_dataFrame.u16_msgCounter |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

        if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_SOURCE_CLIENT_ID) == SMS_PROTOCOL_FLAG_SOURCE_CLIENT_ID) &&
            (s_dataFrame.u8_headerLength > 14)) {
          s_state_Header = SRC_CLIENT_ID_1;
        } else {
          s_state_Header = HEADER_CRC16_H;
        }
        break;
      case SRC_CLIENT_ID_1:
        s_dataFrame.u32_srcClientID |= u8_input << 24;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = SRC_CLIENT_ID_2;
        break;
      case SRC_CLIENT_ID_2:
        s_dataFrame.u32_srcClientID |= u8_input << 16;

        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        if (s_dataFrame.u8_headerLength > 16) {
          s_state_Header = SRC_CLIENT_ID_3;
        } else {
          s_state_Header = HEADER_CRC16_H;
        }
        break;
      case SRC_CLIENT_ID_3:
        s_dataFrame.u32_srcClientID |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = SRC_CLIENT_ID_4;
        break;
      case SRC_CLIENT_ID_4:
        s_dataFrame.u32_srcClientID |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

        if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_DATA_IDENTIFIER) == SMS_PROTOCOL_FLAG_DATA_IDENTIFIER) &&
            (s_dataFrame.u8_headerLength > 18)) {
          s_state_Header = DATA_IDENTIFIER_1;
        } else {
          s_state_Header = HEADER_CRC16_H;
        }
        break;
      case DATA_IDENTIFIER_1:
        // stores data idetifier
        s_dataFrame.u16_dataIdentifier |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = DATA_IDENTIFIER_2;
        break;
      case DATA_IDENTIFIER_2:
        // stores data idetifier
        s_dataFrame.u16_dataIdentifier |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;

        if (((s_dataFrame.u32_flags & SMS_PROTOCOL_FLAG_SEGMENTATION) == SMS_PROTOCOL_FLAG_SEGMENTATION) &&
            (s_dataFrame.u8_headerLength > 20)) {
          s_state_Header = SEGMENTATION_1;
        } else {
          s_state_Header = HEADER_CRC16_H;
        }
        break;
      case SEGMENTATION_1:
        // stores segmentation
        s_dataFrame.u16_segmentation |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = SEGMENTATION_2;
        break;
      case SEGMENTATION_2:
        // stores segmentation
        s_dataFrame.u16_segmentation |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = HEADER_CRC16_H;

        if (s_dataFrame.u16_segmentation - s_dataFrame.u16_msgCounter > 1) {
          s_dataFrame.b_hasNext = true;
        } else {
          s_dataFrame.b_hasNext = false;
        }
        break;
      case HEADER_CRC16_H:
        // store Header CRC16 High Byte
        s_dataFrame.u16_headerCRC16 |= u8_input << 8;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        s_state_Header = HEADER_CRC16_L;
        break;
      case HEADER_CRC16_L: {
        // store Header CRC16 Low Byte
        s_dataFrame.u16_headerCRC16 |= u8_input;
        s_dataFrame.au8_Header[s_dataFrame.u16_indexHeader] = u8_input;
        // verify the Header crc
        uint16_t u16_crc16 = crc16_ccitt_generic(&s_dataFrame.au8_Header[0], s_dataFrame.u8_headerLength - CRC16_LENGTH, CRC_START_CCITT_FFFF);

        if (u16_crc16 == s_dataFrame.u16_headerCRC16) {
          s_state_Header = PAYLOAD_DATA;
        } else {
          //printf("\nHeader CRC16 Error\n");
          s_state_Header = START_PATTERN;
        }
      }
        break;
      case PAYLOAD_DATA:
        // store Payload
        s_dataFrame.au8_Payload[s_dataFrame.u16_indexPayload] = u8_input;
        s_dataFrame.u16_indexPayload++;
        /*
        if (s_dataFrame.u16_indexPayload < s_dataFrame.u16_payloadLength) // read all data into the s_dataFrame.au8_Payload Buffer
        {
        s_dataFrame.au8_Payload[s_dataFrame.u16_indexPayload] = u8_input;
        s_dataFrame.u16_indexPayload++;
        if (s_dataFrame.u16_indexPayload >= s_dataFrame.u16_payloadLength)
        {
            printSMSTransportHeader();
            parse_inputSMSTransportPayload(s_dataFrame.au8_Payload, s_dataFrame.u16_payloadLength);
            s_state_Header = START_PATTERN;
        }
        }
        */
        break;
      default: //do nothing
        break;
    } // switch(s_state_Header)
    s_dataFrame.u16_indexHeader++;
  } // for

  //printSMSTransportHeader();
  if (!s_dataFrame.b_hasNext) {
    parse_inputSMSTransportPayload(s_dataFrame.au8_Payload, s_dataFrame.u16_indexPayload);
  }
  s_state_Header = START_PATTERN;
  s_dataFrame.u16_indexHeader = 0;
  s_dataFrame.u8_startPattern = 0;
  s_dataFrame.u8_protocolVersion = 0;
  s_dataFrame.u8_headerLength = 0;
  s_dataFrame.u16_payloadLength = 0;
  s_dataFrame.u8_AppProtocolType = 0;
  s_dataFrame.u32_flags = 0;
  s_dataFrame.u16_msgCounter = 0;
  s_dataFrame.u32_srcClientID = 0;
  s_dataFrame.u16_dataIdentifier = 0;
  s_dataFrame.u16_segmentation = 0;
  s_dataFrame.u16_headerCRC16 = 0;
}

int main() {
  pcpp::PcapFileReaderDevice reader("../2.pcapng");
  if (!reader.open()) {
    std::cerr << "Error opening the pcap file" << std::endl;
    return 1;
  }

  pcpp::RawPacket rawPacket;
  /*
  if (!reader.getNextPacket(rawPacket)) {
    std::cerr << "Couldn't read the first packet in the file" << std::endl;
    return 1;
  }
  */

  int allCount = 0;
  while (reader.getNextPacket(rawPacket)) {
    pcpp::Packet parsedPacket(&rawPacket);
    if (parsedPacket.isPacketOfType(pcpp::IPv4)) {
      //pcpp::IPv4Address srcIP = parsedPacket.getLayerOfType<pcpp::IPv4Layer>()->getSrcIPv4Address();
      //pcpp::IPv4Address destIP = parsedPacket.getLayerOfType<pcpp::IPv4Layer>()->getDstIPv4Address();
      pcpp::UdpLayer *udp = parsedPacket.getLayerOfType<pcpp::UdpLayer>();
      pcpp::Layer *dataLayer = udp->getNextLayer();

      /*
      if (dataLayer->getDataLen() < 33) {
        std::cout << "Packet length is smaller than 33" << std::endl;
        errCount++;
        continue;
      }
       */

      uint8_t *data = dataLayer->getData();
      parse_inputSMSTransportProtocol(data, dataLayer->getDataLen());
      allCount++;
    }
  }

  std::cout << "Packet count = " << pktCount << std::endl;
  std::cout << "All count = " << allCount << std::endl;

  return 0;
}