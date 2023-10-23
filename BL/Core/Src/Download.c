#include "Download.h"
#include "stdbool.h"
#include "usbd_cdc_if.h"
#include "main.h"
static const uint32_t app_max_size = 80 * 1024;  // 80k
static uint16_t packet_size = 0;
static bool connect_established;
static bool end_of_transmission;
static bool received_begin;
static bool received_data;
static bool received_end;
static bool got_file_disc;
static uint32_t update_file_total_size;
static uint8_t ack_and_c[2] = {ACK, CRC16};

uint8_t wait_second_eot;

void FLASH_If_Init(void)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    HAL_FLASH_Lock();
}

// flash只能按页擦除。第一个参数是待擦除的起始页，第二个参数是要擦除的页数
uint32_t FLASH_If_Erase(uint32_t start_page, uint8_t pages)
{
    uint32_t PageError = 0;
    FLASH_EraseInitTypeDef pEraseInit;
    HAL_StatusTypeDef status = HAL_OK;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    pEraseInit.PageAddress = start_page;
    pEraseInit.Banks = FLASH_BANK_1;
    pEraseInit.NbPages = pages;
    status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);

    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        /* Error occurred while page erase */
        return FLASHIF_ERASEKO;
    }

    return FLASHIF_OK;
}


#define UPDATE_APP_ADDRESS     (uint32_t)ApplicationAddress      /* ADDR_FLASH_PAGE_88 */
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
    uint32_t i = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    for (i = 0; (i < length) && (destination <= (USER_FLASH_END_ADDRESS-4)); i++)
    {
        /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
           be done by word */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destination, *(uint32_t*)(p_source+i)) == HAL_OK)
        {
            /* Check the written value */
            if (*(uint32_t*)destination != *(uint32_t*)(p_source+i))
            {
                /* Flash content doesn't match SRAM content */
                return(FLASHIF_WRITINGCTRL_ERROR);
            }
            /* Increment FLASH destination address */
            destination += 4;
        }
        else
        {
            /* Error occurred while writing data in Flash memory */
            return (FLASHIF_WRITING_ERROR);
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return (FLASHIF_OK);
}


static void ClearFlags(void) {
    connect_established = false;
    got_file_disc = false;
    end_of_transmission = false;
    wait_second_eot = 0;
}

void PutByte(uint8_t byte) {
    CDC_Transmit_FS(&byte, 1);
}

void YmodemHandshakeCb() {
    if (!connect_established) {
        PutByte(CRC16);
    }

    if (end_of_transmission) {
        wait_second_eot++;
        // 没有收到第二个EOT
        if (wait_second_eot >= 5) {
            ClearFlags();
        }
    }
}

static uint16_t CalculateCrc16(unsigned char *q, int len)
{
    uint16_t crc;
    char i;

    crc = 0;
    while (--len >= 0)
    {
        crc = crc ^ (int) * q++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        }
        while (--i);
    }

    return (crc);
}

static void PutBytes(uint8_t* bytes, uint8_t len) {
    CDC_Transmit_FS(bytes, len);
}

/**
 * @brief 终止传输
 *
 */
static void TransAbort(void) {
    uint8_t tmp[2] = {CA, CA};
    PutBytes(tmp, 2);
}

enum {
    CHECK_HEAD,
    COPY_PACKET,
    CHECK_CRC
};

uint8_t ymodem_packet[1029];

void YmodemPacketHandle(void) {
    static uint32_t current_write_size = 0;
    static uint32_t write_ptr = 0;

    // 收到传输开始包
    if (received_begin) {
        received_begin = false;
        if (!got_file_disc) {
            char* file_name = (char*)&ymodem_packet[PACKET_DATA_INDEX];
            char* file_size = (char*)&ymodem_packet[PACKET_DATA_INDEX + strlen(file_name) + 1];
            update_file_total_size = atol(file_size);

            if (update_file_total_size > app_max_size) {
                // abort transmission
                TransAbort();
            } else {
                // Unlocks Flash for write access
                FLASH_If_Init();
                // erase app area
                FLASH_If_Erase(UPDATE_APP_ADDRESS, update_file_total_size / FLASH_PAGE_SIZE + 1);
                current_write_size = 0;
                write_ptr = UPDATE_APP_ADDRESS;
                PutBytes(ack_and_c, 2);
            }

            got_file_disc = true;
        }
    }

    // 收到正常数据包
    if (received_data) {
        received_data = false;
        if (current_write_size + packet_size > update_file_total_size) {
            if (FLASH_If_Write(write_ptr, (uint32_t*)&ymodem_packet[PACKET_DATA_INDEX],
                               (update_file_total_size - current_write_size) / 4) == FLASHIF_OK) {
                write_ptr += packet_size;
                current_write_size += packet_size;
                PutByte(ACK);
            } else {
                TransAbort();
            }
        } else {
            if (FLASH_If_Write(write_ptr, (uint32_t*)&ymodem_packet[PACKET_DATA_INDEX], packet_size / 4)
                == FLASHIF_OK) {
                write_ptr += packet_size;
                current_write_size += packet_size;
                PutByte(ACK);
            } else {
                TransAbort();
            }
        }
    }

    // 收到结束包
    if (received_end) {
        received_end = false;
        if (FLASH_If_Write(UPDATE_APP_ADDRESS, &update_file_total_size, 1) == FLASHIF_OK) {
            // reset mcu
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    }
}
/**
 * @brief USB虚拟串口接收回调。对CDC包解析，并解析到的完整的ymodem包放入全局缓存-ymodem_packet
 *
 * @param packet 数据包的地址
 * @param len 收到的长度
 * @return uint8_t
 */
uint8_t PacketParse(uint8_t* packet, uint8_t len) {
    uint32_t crc;
    uint32_t crc_local;
    uint8_t char1;

    static uint8_t state = CHECK_HEAD;
    static uint16_t index = 0;

    // CDC一包最大64字节
    switch (state) {
        case CHECK_HEAD:
            char1 = packet[0];
            switch (char1) {
                case SOH:
                    packet_size = PACKET_SIZE;
                    index = 0;
                    memcpy(ymodem_packet, packet, len);
                    index += len;
                    state = COPY_PACKET;
                    break;
                case STX:
                    packet_size = PACKET_1K_SIZE;
                    index = 0;
                    // copy the first packet;
                    memcpy(ymodem_packet, packet, len);
                    index += len;
                    state = COPY_PACKET;
                    break;
                case EOT:
                    if (!end_of_transmission) {
                        PutByte(NAK);
                        end_of_transmission = true;
                    } else {
                        PutBytes(ack_and_c, 2);
                    }
                    break;
                case CA:
                    break;
                case ABORT1:
                case ABORT2:
                    break;

                default:
                    break;
            }
            break;

        case COPY_PACKET:
            memcpy(&ymodem_packet[index], packet, len);
            index += len;
            if (index == packet_size + 5) {
                crc = ymodem_packet[PACKET_DATA_INDEX + packet_size] << 8;
                crc += ymodem_packet[PACKET_DATA_INDEX + packet_size + 1];

                crc_local = CalculateCrc16(&ymodem_packet[PACKET_DATA_INDEX], packet_size);
                if (crc_local == crc) {
                    state = CHECK_HEAD;

                    if (ymodem_packet[PACKET_START_INDEX] == SOH) {
                        if (ymodem_packet[PACKET_NUMBER_INDEX] == 0) {
                            if (end_of_transmission) {
                                PutByte(ACK);
                                received_end = true;
                                ClearFlags();
                            } else {
                                connect_established = true;
                                received_begin = true;
                            }
                        } else {
                            // 小于128字节的数据包
                            received_data = true;
                        }
                    } else {
                        received_data = true;  // normal data packet
                    }
                } else {
                    state = CHECK_HEAD;
                }
            }
            break;
    }

    return 0;
}
