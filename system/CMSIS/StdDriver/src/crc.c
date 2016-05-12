/**************************************************************************//**
 * @file     crc.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/07/02 11:21a $
 * @brief    NUC123 series CRC driver source file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NUC123.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CRC_Driver CRC Driver
  @{
*/

/** @addtogroup CRC_EXPORTED_FUNCTIONS CRC Exported Functions
  @{
*/

/**
  * @brief      CRC Open
  *
  * @param[in]  u32Mode         CRC operation polynomial mode. Valid values are:
  *                             - \ref CRC_CCITT
  *                             - \ref CRC_8
  *                             - \ref CRC_16
  *                             - \ref CRC_32
  * @param[in]  u32Attribute    CRC operation data attribute. Valid values are combined with:
  *                             - \ref CRC_CHECKSUM_COM
  *                             - \ref CRC_CHECKSUM_RVS
  *                             - \ref CRC_WDATA_COM
  *                             - \ref CRC_WDATA_RVS
  * @param[in]  u32Seed         Seed value.
  * @param[in]  u32DataLen      CPU Write Data Length. Valid values are:
  *                             - \ref CRC_CPU_WDATA_8
  *                             - \ref CRC_CPU_WDATA_16
  *                             - \ref CRC_CPU_WDATA_32
  *
  * @return     None
  *
  * @details    This function enable the CRC channel by specify CRC polynomial mode, data attribute, initial seed and write data length.
  */
void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen)
{
    /* Enable CRC channel clock */
    PDMA_GCR->GCRCSR |= PDMA_GCRCSR_CRC_CLK_EN_Msk;

    CRC->SEED = u32Seed;
    CRC->CTL = u32Mode | u32Attribute | u32DataLen | CRC_CTL_CRCCEN_Msk;

    /* Setting RST bit will reload the initial seed value (CRC_SEED register) */
    CRC->CTL |= CRC_CTL_CRC_RST_Msk;
}

/**
  * @brief      CRC Start DMA transfer
  *
  * @param[in]  u32SrcAddr      Starting source address of CRC DMA transfer.
  * @param[in]  u32ByteCount    Calculate byte counts of CRC DMA transfer.
  *
  * @return     None
  *
  * @details    This function start CRC DMA transfer from specify source address and byte counts.
  */
void CRC_StartDMATransfer(uint32_t u32SrcAddr, uint32_t u32ByteCount)
{
    CRC->DMASAR = u32SrcAddr;
    CRC->DMABCR = u32ByteCount;
    CRC->CTL |= CRC_CTL_TRIG_EN_Msk;
}

/**
  * @brief      Get CRC Checksum
  *
  * @param      None
  *
  * @return     Checksum Value
  *
  * @details    This macro get the CRC checksum result by current CRC polynomial mode.
  */
uint32_t CRC_GetChecksum(void)
{
    switch(CRC->CTL & CRC_CTL_CRC_MODE_Msk)
    {
        case CRC_CCITT:
        case CRC_16:
            return (CRC->CHECKSUM & 0xFFFF);

        case CRC_32:
            return (CRC->CHECKSUM);

        case CRC_8:
            return (CRC->CHECKSUM & 0xFF);

        default:
            return 0;
    }
}

/*@}*/ /* end of group CRC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CRC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
