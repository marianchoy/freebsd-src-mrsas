/*
 * Copyright (c) 2012, LSI Corp.
 * All rights reserved.
 * Author: Marian Choy
 * Support: freebsdraid@lsi.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the <ORGANIZATION> nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing
 * official policies,either expressed or implied, of the FreeBSD Project.
 *
 * Send feedback to: <megaraidfbsd@lsi.com>
 * Mail to: LSI Corporation, 1621 Barber Lane, Milpitas, CA 95035
 *    ATTN: MegaRaid FreeBSD
 *
 */

#include <dev/mrsas/mrsas.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/cam_sim.h>
#include <cam/cam_xpt_sim.h>
#include <cam/cam_debug.h>
#include <cam/cam_periph.h>
#include <cam/cam_xpt_periph.h>


/*
 * Function prototypes
 */
u_int8_t MR_ValidateMapInfo(struct mrsas_softc *sc);
u_int8_t mrsas_get_best_arm(PLD_LOAD_BALANCE_INFO lbInfo, u_int8_t arm, 
       u_int64_t block, u_int32_t count);
u_int8_t MR_BuildRaidContext(struct mrsas_softc *sc, 
        struct IO_REQUEST_INFO *io_info,
        RAID_CONTEXT *pRAID_Context, MR_FW_RAID_MAP_ALL *map);
u_int8_t MR_GetPhyParams(struct mrsas_softc *sc, u_int32_t ld, 
        u_int64_t stripRow, u_int16_t stripRef, u_int64_t *pdBlock,
        u_int16_t *pDevHandle,  RAID_CONTEXT *pRAID_Context, 
        MR_FW_RAID_MAP_ALL *map);
u_int16_t MR_TargetIdToLdGet(u_int32_t ldTgtId, MR_FW_RAID_MAP_ALL *map);
u_int16_t MR_GetLDTgtId(u_int32_t ld, MR_FW_RAID_MAP_ALL *map);
u_int16_t mrsas_get_updated_dev_handle(PLD_LOAD_BALANCE_INFO lbInfo, 
        struct IO_REQUEST_INFO *io_info);
u_int32_t mega_mod64(u_int64_t dividend, u_int32_t divisor);
u_int32_t MR_GetSpanBlock(u_int32_t ld, u_int64_t row, u_int64_t *span_blk, 
        MR_FW_RAID_MAP_ALL *map, int *div_error);
u_int64_t mega_div64_32(u_int64_t dividend, u_int32_t divisor);
void mrsas_update_load_balance_params(MR_FW_RAID_MAP_ALL *map, 
        PLD_LOAD_BALANCE_INFO lbInfo);
void mrsas_set_pd_lba(MRSAS_RAID_SCSI_IO_REQUEST *io_request, 
        u_int8_t cdb_len, struct IO_REQUEST_INFO *io_info, union ccb *ccb,
        MR_FW_RAID_MAP_ALL *local_map_ptr, u_int32_t ref_tag);
static u_int16_t MR_LdSpanArrayGet(u_int32_t ld, u_int32_t span, 
        MR_FW_RAID_MAP_ALL *map);
static u_int16_t MR_PdDevHandleGet(u_int32_t pd, MR_FW_RAID_MAP_ALL *map);
static u_int16_t MR_ArPdGet(u_int32_t ar, u_int32_t arm, 
        MR_FW_RAID_MAP_ALL *map);
static MR_LD_SPAN *MR_LdSpanPtrGet(u_int32_t ld, u_int32_t span, 
        MR_FW_RAID_MAP_ALL *map);
static u_int8_t MR_LdDataArmGet(u_int32_t ld, u_int32_t armIdx, 
        MR_FW_RAID_MAP_ALL *map);
static MR_SPAN_BLOCK_INFO *MR_LdSpanInfoGet(u_int32_t ld, 
        MR_FW_RAID_MAP_ALL *map);
MR_LD_RAID *MR_LdRaidGet(u_int32_t ld, MR_FW_RAID_MAP_ALL *map);


/*
 * Related Defines
 */

typedef u_int64_t  REGION_KEY;
typedef u_int32_t  REGION_LEN;

#define MR_LD_STATE_OPTIMAL 3
#define FALSE 0
#define TRUE 1


/*
 * Related Macros
 */

#define ABS_DIFF(a,b)   ( ((a) > (b)) ? ((a) - (b)) : ((b) - (a)) )

#define swap32(x) \
  ((unsigned int)( \
    (((unsigned int)(x) & (unsigned int)0x000000ffUL) << 24) | \
    (((unsigned int)(x) & (unsigned int)0x0000ff00UL) <<  8) | \
    (((unsigned int)(x) & (unsigned int)0x00ff0000UL) >>  8) | \
    (((unsigned int)(x) & (unsigned int)0xff000000UL) >> 24) ))


/*
 * In-line functions for mod and divide of 64-bit dividend and 32-bit divisor.
 * Assumes a check for a divisor of zero is not possible. 
 * 
 * @param dividend   : Dividend
 * @param divisor    : Divisor
 * @return remainder
 */

inline u_int32_t mega_mod64(u_int64_t dividend, u_int32_t divisor)
{
    return (dividend % divisor);
}
#define mega_mod64(dividend, divisor) ({ \
int remainder; \
remainder = ((u_int64_t) (dividend)) % (u_int32_t) (divisor); \
remainder;})

inline u_int64_t mega_div64_32(u_int64_t dividend, u_int32_t divisor)
{
    return (dividend / divisor);
}
#define mega_div64_32(dividend, divisor) ({ \
int quotient; \
quotient = ((u_int64_t) (dividend)) / (u_int32_t) (divisor); \
quotient;})


/*
 * Various RAID map access functions.  These functions access the various
 * parts of the RAID map and returns the appropriate parameters. 
 */

MR_LD_RAID *MR_LdRaidGet(u_int32_t ld, MR_FW_RAID_MAP_ALL *map)
{
    return (&map->raidMap.ldSpanMap[ld].ldRaid);
}

u_int16_t MR_GetLDTgtId(u_int32_t ld, MR_FW_RAID_MAP_ALL *map)
{
    return (map->raidMap.ldSpanMap[ld].ldRaid.targetId);
}

static u_int16_t MR_LdSpanArrayGet(u_int32_t ld, u_int32_t span, MR_FW_RAID_MAP_ALL *map)
{
    return map->raidMap.ldSpanMap[ld].spanBlock[span].span.arrayRef;
}

static u_int8_t MR_LdDataArmGet(u_int32_t ld, u_int32_t armIdx, MR_FW_RAID_MAP_ALL *map)
{
    return map->raidMap.ldSpanMap[ld].dataArmMap[armIdx];
}

static u_int16_t MR_PdDevHandleGet(u_int32_t pd, MR_FW_RAID_MAP_ALL *map)
{
    return map->raidMap.devHndlInfo[pd].curDevHdl;
}

static u_int16_t MR_ArPdGet(u_int32_t ar, u_int32_t arm, MR_FW_RAID_MAP_ALL *map)
{
    return map->raidMap.arMapInfo[ar].pd[arm];
}

static MR_LD_SPAN *MR_LdSpanPtrGet(u_int32_t ld, u_int32_t span, MR_FW_RAID_MAP_ALL *map)
{
    return &map->raidMap.ldSpanMap[ld].spanBlock[span].span;
}

static MR_SPAN_BLOCK_INFO *MR_LdSpanInfoGet(u_int32_t ld, MR_FW_RAID_MAP_ALL *map)
{
    return &map->raidMap.ldSpanMap[ld].spanBlock[0];
}

u_int16_t MR_TargetIdToLdGet(u_int32_t ldTgtId, MR_FW_RAID_MAP_ALL *map)
{
    return map->raidMap.ldTgtIdToLd[ldTgtId];
}

/**
 * MR_ValidateMapInfo:        Validate RAID map
 * input:                     Adapter instance soft state
 *
 * This function checks and validates the loaded RAID map. It returns 0 if 
 * successful, and 1 otherwise.
 */
u_int8_t MR_ValidateMapInfo(struct mrsas_softc *sc)
{
    uint32_t total_map_sz;
    MR_FW_RAID_MAP_ALL *map = sc->raidmap_mem[(sc->map_id & 1)];
    MR_FW_RAID_MAP *pFwRaidMap = &map->raidMap;

    total_map_sz = (sizeof(MR_FW_RAID_MAP) - sizeof(MR_LD_SPAN_MAP) +
                     (sizeof(MR_LD_SPAN_MAP) * pFwRaidMap->ldCount));

    if (pFwRaidMap->totalSize != total_map_sz) {
        device_printf(sc->mrsas_dev, "map size %x not matching ld count\n", total_map_sz);
        device_printf(sc->mrsas_dev, "span map= %x\n", (unsigned int)sizeof(MR_LD_SPAN_MAP));
        device_printf(sc->mrsas_dev, "pFwRaidMap->totalSize=%x\n", pFwRaidMap->totalSize);
        return 1;
    }
    mrsas_update_load_balance_params(map, sc->load_balance_info);

    return 0;
}

/**
* MR_BuildRaidContext:           Set up Fast path RAID context
*
* This function will initiate command processing.  The start/end row 
* and strip information is calculated then the lock is acquired.
* This function will return 0 if region lock was acquired OR return 
* num strips.
*/
u_int8_t 
MR_BuildRaidContext(struct mrsas_softc *sc, struct IO_REQUEST_INFO *io_info,
                    RAID_CONTEXT *pRAID_Context, MR_FW_RAID_MAP_ALL *map)
{
    MR_LD_RAID *raid;
    u_int32_t ld, stripSize, stripe_mask;
    u_int64_t endLba, endStrip, endRow, start_row, start_strip;
    REGION_KEY regStart;
    REGION_LEN regSize;
    u_int8_t num_strips, numRows;
    u_int16_t ref_in_start_stripe, ref_in_end_stripe;
    u_int64_t ldStartBlock;
    u_int32_t numBlocks, ldTgtId;
    u_int8_t isRead, stripIdx;
    u_int8_t retval = 0;

    ldStartBlock = io_info->ldStartBlock;
    numBlocks = io_info->numBlocks;
    ldTgtId = io_info->ldTgtId;
    isRead = io_info->isRead;

    ld = MR_TargetIdToLdGet(ldTgtId, map);
    raid = MR_LdRaidGet(ld, map);

    stripSize = 1 << raid->stripeShift;
    stripe_mask = stripSize-1;
    /*
     * calculate starting row and stripe, and number of strips and rows
     */
    start_strip = ldStartBlock >> raid->stripeShift;
    ref_in_start_stripe = (u_int16_t)(ldStartBlock & stripe_mask);
    endLba = ldStartBlock + numBlocks - 1;
    ref_in_end_stripe = (u_int16_t)(endLba & stripe_mask);
    endStrip = endLba >> raid->stripeShift;
    num_strips = (u_int8_t)(endStrip - start_strip + 1);     // End strip
    if (raid->rowDataSize == 0)
        return FALSE;
    start_row = mega_div64_32(start_strip, raid->rowDataSize);// Start Row
    endRow = mega_div64_32(endStrip, raid->rowDataSize);
    numRows = (u_int8_t)(endRow - start_row + 1);   // get the row count

    /*
     * Calculate region info.  (Assume region at start of first row, and 
     * assume this IO needs the full row - will adjust if not true.)
     */
    regStart = start_row << raid->stripeShift;  
    regSize = stripSize;                       

    /* Check if we can send this I/O via FastPath */
    if (raid->capability.fpCapable) {
        if (isRead)
            io_info->fpOkForIo = (raid->capability.fpReadCapable &&
                                              ((num_strips == 1) ||
                                               raid->capability.
                                               fpReadAcrossStripe));
        else
            io_info->fpOkForIo = (raid->capability.fpWriteCapable &&
                                              ((num_strips == 1) ||
                                               raid->capability.
                                               fpWriteAcrossStripe));
    } 
    else
        io_info->fpOkForIo = FALSE;

    if (numRows == 1) {
        if (num_strips == 1) {  
            /* single-strip IOs can always lock only the data needed,
               multi-strip IOs always need to full stripe locked */ 
            regStart += ref_in_start_stripe;
            regSize = numBlocks;
        }  
    } 
    else {
        /* if the start strip is the last in the start row */
        if (start_strip == (start_row + 1) * raid->rowDataSize - 1) {  
            regStart += ref_in_start_stripe;
            /* initialize count to sectors from startRef to end of strip */
            regSize = stripSize - ref_in_start_stripe;
        }

        if (numRows > 2)
            regSize += (numRows-2) << raid->stripeShift; // add complete rows 
                                                         // in middle of transfer
        /* If IO ends within first strip of last row */
        if (endStrip == endRow*raid->rowDataSize)     
            regSize += ref_in_end_stripe+1;
        else
            regSize += stripSize;
    }
    pRAID_Context->timeoutValue = map->raidMap.fpPdIoTimeoutSec;
    pRAID_Context->regLockFlags = (isRead)? REGION_TYPE_SHARED_READ : raid->regTypeReqOnWrite;
    pRAID_Context->VirtualDiskTgtId = raid->targetId;
    pRAID_Context->regLockRowLBA = regStart;
    pRAID_Context->regLockLength = regSize;
    pRAID_Context->configSeqNum = raid->seqNum;

    /*
     * Get Phy Params only if FP capable, or else leave it to MR firmware 
     * to do the calculation.
     */
    if (io_info->fpOkForIo) {
        retval = MR_GetPhyParams(sc, ld, start_strip, ref_in_start_stripe, 
                 &io_info->pdBlock, &io_info->devHandle, pRAID_Context, map);
        /* If IO on an invalid Pd, then FP is not possible */
        if (io_info->devHandle == MR_PD_INVALID) 
            io_info->fpOkForIo = FALSE;
        return retval;
    } 
    else if (isRead) {
        for (stripIdx=0; stripIdx<num_strips; stripIdx++) {
            if (!MR_GetPhyParams(sc, ld, start_strip + stripIdx, 
                    ref_in_start_stripe, &io_info->pdBlock, 
                    &io_info->devHandle, pRAID_Context, map))
                return TRUE;
        }
    }
    return TRUE;
}

/**
 * mrsas_update_load_balance_params:  Update load balance parmas  
 * Inputs:                         map pointer 
 *                                 Load balance info 
 *                                 io_info pointer
 *
 * This function updates the load balance parameters for the LD config
 * of a two drive optimal RAID-1.  
 */
void mrsas_update_load_balance_params(MR_FW_RAID_MAP_ALL *map, 
        PLD_LOAD_BALANCE_INFO lbInfo)
{
    int ldCount;
    u_int16_t ld;
    u_int32_t pd, arRef;
    MR_LD_RAID *raid;

    for (ldCount = 0; ldCount < MAX_LOGICAL_DRIVES; ldCount++)
    {
        ld = MR_TargetIdToLdGet(ldCount, map);
        if (ld >= MAX_LOGICAL_DRIVES) {
            lbInfo[ldCount].loadBalanceFlag = 0;
            continue;
        }

        raid = MR_LdRaidGet(ld, map);

        /* Two drive Optimal RAID 1 */
        if ((raid->level == 1) && (raid->rowSize == 2) && 
                (raid->spanDepth == 1)
                && raid->ldState == MR_LD_STATE_OPTIMAL) {
            lbInfo[ldCount].loadBalanceFlag = 1;

            /* Get the array on which this span is present */
            arRef = MR_LdSpanArrayGet(ld, 0, map);    

            /* Get the PD */
            pd = MR_ArPdGet(arRef, 0, map);      
            /* Get dev handle from PD */
            lbInfo[ldCount].raid1DevHandle[0] = MR_PdDevHandleGet(pd, map);       
            pd = MR_ArPdGet(arRef, 1, map);     
            lbInfo[ldCount].raid1DevHandle[1] = MR_PdDevHandleGet(pd, map);        
        } 
        else
            lbInfo[ldCount].loadBalanceFlag = 0;
    }
}


/**
 * mrsas_set_pd_lba:    Sets PD LBA
 * input:               io_request pointer
 *                      CDB length
 *                      io_info pointer
 *                      Pointer to CCB
 *                      Local RAID map pointer
 *                      Start block of IO
 *
 * Used to set the PD logical block address in CDB for FP IOs.
 */
void mrsas_set_pd_lba(MRSAS_RAID_SCSI_IO_REQUEST *io_request, u_int8_t cdb_len,
    struct IO_REQUEST_INFO *io_info, union ccb *ccb,
    MR_FW_RAID_MAP_ALL *local_map_ptr, u_int32_t ref_tag)
{
    MR_LD_RAID *raid;
    u_int32_t ld;
    u_int64_t start_blk = io_info->pdBlock;
    u_int8_t *cdb = io_request->CDB.CDB32;
    u_int32_t num_blocks = io_info->numBlocks;
    u_int8_t opcode = 0, flagvals = 0, groupnum = 0, control = 0;
    struct ccb_hdr *ccb_h = &(ccb->ccb_h);

    /* Check if T10 PI (DIF) is enabled for this LD */
    ld = MR_TargetIdToLdGet(io_info->ldTgtId, local_map_ptr);
    raid = MR_LdRaidGet(ld, local_map_ptr);
    if (raid->capability.ldPiMode == MR_PROT_INFO_TYPE_CONTROLLER) {
        memset(cdb, 0, sizeof(io_request->CDB.CDB32));
        cdb[0] =  MRSAS_SCSI_VARIABLE_LENGTH_CMD;
        cdb[7] =  MRSAS_SCSI_ADDL_CDB_LEN;

        if (ccb_h->flags == CAM_DIR_OUT)
            cdb[9] = MRSAS_SCSI_SERVICE_ACTION_READ32;
        else
            cdb[9] = MRSAS_SCSI_SERVICE_ACTION_WRITE32;
        cdb[10] = MRSAS_RD_WR_PROTECT_CHECK_ALL;

        /* LBA */
        cdb[12] = (u_int8_t)((start_blk >> 56) & 0xff);
        cdb[13] = (u_int8_t)((start_blk >> 48) & 0xff);
        cdb[14] = (u_int8_t)((start_blk >> 40) & 0xff);
        cdb[15] = (u_int8_t)((start_blk >> 32) & 0xff);
        cdb[16] = (u_int8_t)((start_blk >> 24) & 0xff);
        cdb[17] = (u_int8_t)((start_blk >> 16) & 0xff);
        cdb[18] = (u_int8_t)((start_blk >> 8) & 0xff);
        cdb[19] = (u_int8_t)(start_blk & 0xff);

        /* Logical block reference tag */
        io_request->CDB.EEDP32.PrimaryReferenceTag = swap32(ref_tag);
        io_request->CDB.EEDP32.PrimaryApplicationTagMask = 0xffff;
        io_request->IoFlags = 32; /* Specify 32-byte cdb */

        /* Transfer length */
        cdb[28] = (u_int8_t)((num_blocks >> 24) & 0xff);
        cdb[29] = (u_int8_t)((num_blocks >> 16) & 0xff);
        cdb[30] = (u_int8_t)((num_blocks >> 8) & 0xff);
        cdb[31] = (u_int8_t)(num_blocks & 0xff);

        /* set SCSI IO EEDP Flags */
        if (ccb_h->flags == CAM_DIR_OUT) {
            io_request->EEDPFlags =
                MPI2_SCSIIO_EEDPFLAGS_INC_PRI_REFTAG  |
                MPI2_SCSIIO_EEDPFLAGS_CHECK_REFTAG |
                MPI2_SCSIIO_EEDPFLAGS_CHECK_REMOVE_OP |
                MPI2_SCSIIO_EEDPFLAGS_CHECK_APPTAG |
                MPI2_SCSIIO_EEDPFLAGS_CHECK_GUARD;
        } 
        else {
                io_request->EEDPFlags =
                     MPI2_SCSIIO_EEDPFLAGS_INC_PRI_REFTAG |
                     MPI2_SCSIIO_EEDPFLAGS_INSERT_OP;
        }
        io_request->Control |= (0x4 << 26);
        io_request->EEDPBlockSize = MRSAS_EEDPBLOCKSIZE;
    } 
    else {
        /* Some drives don't support 16/12 byte CDB's, convert to 10 */
        if (((cdb_len == 12) || (cdb_len == 16)) && 
                    (start_blk <= 0xffffffff)) {
            if (cdb_len == 16) {
                opcode = cdb[0] == READ_16 ? READ_10 : WRITE_10;
                flagvals = cdb[1];
                groupnum = cdb[14];
                control = cdb[15];
            } 
            else {
                opcode = cdb[0] == READ_12 ? READ_10 : WRITE_10;
                flagvals = cdb[1];
                groupnum = cdb[10];
                control = cdb[11];
            }

            memset(cdb, 0, sizeof(io_request->CDB.CDB32));

            cdb[0] = opcode;
            cdb[1] = flagvals;
            cdb[6] = groupnum;
            cdb[9] = control;

            /* Transfer length */
            cdb[8] = (u_int8_t)(num_blocks & 0xff);
            cdb[7] = (u_int8_t)((num_blocks >> 8) & 0xff);

            io_request->IoFlags = 10; /* Specify 10-byte cdb */
            cdb_len = 10;
        } else if ((cdb_len < 16) && (start_blk > 0xffffffff)) {
            /* Convert to 16 byte CDB for large LBA's */
            switch (cdb_len) {
                case 6:
                    opcode = cdb[0] == READ_6 ? READ_16 : WRITE_16;
                    control = cdb[5];
                    break;
                case 10:
                    opcode = cdb[0] == READ_10 ? READ_16 : WRITE_16;
                    flagvals = cdb[1];
                    groupnum = cdb[6];
                    control = cdb[9];
                    break;
                case 12:
                    opcode = cdb[0] == READ_12 ? READ_16 : WRITE_16;
                    flagvals = cdb[1];
                    groupnum = cdb[10];
                    control = cdb[11];
                    break;
            }

            memset(cdb, 0, sizeof(io_request->CDB.CDB32));

            cdb[0] = opcode;
            cdb[1] = flagvals;
            cdb[14] = groupnum;
            cdb[15] = control;

            /* Transfer length */
            cdb[13] = (u_int8_t)(num_blocks & 0xff);
            cdb[12] = (u_int8_t)((num_blocks >> 8) & 0xff);
            cdb[11] = (u_int8_t)((num_blocks >> 16) & 0xff);
            cdb[10] = (u_int8_t)((num_blocks >> 24) & 0xff);

            io_request->IoFlags = 16; /* Specify 16-byte cdb */
            cdb_len = 16;
        } else if ((cdb_len == 6) && (start_blk > 0x1fffff)) {
            /* convert to 10 byte CDB */
	    opcode = cdb[0] == READ_6 ? READ_10 : WRITE_10;
	    control = cdb[5];
		
	    memset(cdb, 0, sizeof(cdb));
	    cdb[0] = opcode;
	    cdb[9] = control;

	    /* Set transfer length */
	    cdb[8] = (u_int8_t)(num_blocks & 0xff);
	    cdb[7] = (u_int8_t)((num_blocks >> 8) & 0xff);

	    /* Specify 10-byte cdb */
	    cdb_len = 10;
	}

        /* Fall through normal case, just load LBA here */
        switch (cdb_len)
        {
            case 6:
            {
                u_int8_t val = cdb[1] & 0xE0;
                cdb[3] = (u_int8_t)(start_blk & 0xff);
                cdb[2] = (u_int8_t)((start_blk >> 8) & 0xff);
                cdb[1] = val | ((u_int8_t)(start_blk >> 16) & 0x1f);
                break;
            }
            case 10:
                cdb[5] = (u_int8_t)(start_blk & 0xff);
                cdb[4] = (u_int8_t)((start_blk >> 8) & 0xff);
                cdb[3] = (u_int8_t)((start_blk >> 16) & 0xff);
                cdb[2] = (u_int8_t)((start_blk >> 24) & 0xff);
                break;
            case 12:
                cdb[5] = (u_int8_t)(start_blk & 0xff);
                cdb[4] = (u_int8_t)((start_blk >> 8) & 0xff);
                cdb[3] = (u_int8_t)((start_blk >> 16) & 0xff);
                cdb[2] = (u_int8_t)((start_blk >> 24) & 0xff);
                break;
            case 16:
                cdb[9] = (u_int8_t)(start_blk & 0xff);
                cdb[8] = (u_int8_t)((start_blk >> 8) & 0xff);
                cdb[7] = (u_int8_t)((start_blk >> 16) & 0xff);
                cdb[6] = (u_int8_t)((start_blk >> 24) & 0xff);
                cdb[5] = (u_int8_t)((start_blk >> 32) & 0xff);
                cdb[4] = (u_int8_t)((start_blk >> 40) & 0xff);
                cdb[3] = (u_int8_t)((start_blk >> 48) & 0xff);
                cdb[2] = (u_int8_t)((start_blk >> 56) & 0xff);
                break;
        }
    }
}

/**
 * mrsas_get_best_arm         Determine the best spindle arm  
 * Inputs:                    Load balance info 
 *
 * This function determines and returns the best arm by looking at the
 * parameters of the last PD access.
 */
u_int8_t mrsas_get_best_arm(PLD_LOAD_BALANCE_INFO lbInfo, u_int8_t arm, 
        u_int64_t block, u_int32_t count)
{
    u_int16_t     pend0, pend1;
    u_int64_t     diff0, diff1;
    u_int8_t      bestArm;

    /* get the pending cmds for the data and mirror arms */
    pend0 = atomic_read(&lbInfo->scsi_pending_cmds[0]);
    pend1 = atomic_read(&lbInfo->scsi_pending_cmds[1]);

    /* Determine the disk whose head is nearer to the req. block */
    diff0 = ABS_DIFF(block, lbInfo->last_accessed_block[0]);
    diff1 = ABS_DIFF(block, lbInfo->last_accessed_block[1]);
    bestArm = (diff0 <= diff1 ? 0 : 1);

    if ((bestArm == arm && pend0 > pend1 + 16) || (bestArm != arm && pend1 > pend0 + 16))
        bestArm ^= 1;

    /* Update the last accessed block on the correct pd */
    lbInfo->last_accessed_block[bestArm] = block + count - 1;

    return bestArm;
}

/**
 * mrsas_get_updated_dev_handle    Get the update dev handle  
 * Inputs:                         Load balance info 
 *                                 io_info pointer
 *
 * This function determines and returns the updated dev handle.
 */
u_int16_t mrsas_get_updated_dev_handle(PLD_LOAD_BALANCE_INFO lbInfo, 
        struct IO_REQUEST_INFO *io_info)
{
    u_int8_t arm, old_arm;
    u_int16_t devHandle;

    old_arm = lbInfo->raid1DevHandle[0] == io_info->devHandle ? 0 : 1;

    /* get best new arm */
    arm  = mrsas_get_best_arm(lbInfo, old_arm, io_info->ldStartBlock, io_info->numBlocks);
    devHandle = lbInfo->raid1DevHandle[arm];
    atomic_inc(&lbInfo->scsi_pending_cmds[arm]);

    return devHandle;
}

/**
 * MR_GetPhyParams     Calculates arm, span, and block
 * Inputs:             Adapter instance soft state 
 *                     Logical drive number (LD) 
 *                     Stripe number (stripRow)
 *                     Reference in stripe (stripRef)
 * Outputs:            Span number
 *                     Absolute Block number in the physical disk 
 *
 * This routine calculates the arm, span and block for the specified stripe
 * and reference in stripe.
 */
u_int8_t MR_GetPhyParams(struct mrsas_softc *sc, u_int32_t ld, 
            u_int64_t stripRow,
            u_int16_t stripRef, u_int64_t *pdBlock, u_int16_t *pDevHandle,
            RAID_CONTEXT *pRAID_Context, MR_FW_RAID_MAP_ALL *map)
{
    MR_LD_RAID  *raid = MR_LdRaidGet(ld, map);
    u_int32_t pd, arRef;
    u_int8_t physArm, span;
    u_int64_t row;
    u_int8_t retval = TRUE;
    int error_code = 0;
    u_int32_t rowMod, armQ, arm, logArm;

    row =  mega_div64_32(stripRow, raid->rowDataSize);

    if (raid->level == 6) {
        logArm = mega_mod64(stripRow, raid->rowDataSize); // logical arm within row
        if (raid->rowSize == 0)
            return FALSE;
        rowMod = mega_mod64(row, raid->rowSize);  // get logical row mod
        armQ = raid->rowSize-1-rowMod;  // index of Q drive
        arm = armQ+1+logArm;    // data always logically follows Q
        if (arm >= raid->rowSize)         // handle wrap condition
            arm -= raid->rowSize;
        physArm = (u_int8_t)arm;
    } 
    else {
        if (raid->modFactor == 0)
            return FALSE;
        physArm = MR_LdDataArmGet(ld, mega_mod64(stripRow, raid->modFactor), map);
    }

    if (raid->spanDepth == 1) {
        span = 0;
        *pdBlock = row << raid->stripeShift;
    } 
    else {
        span = (u_int8_t)MR_GetSpanBlock(ld, row, pdBlock, map, &error_code);
        if (error_code == 1)
            return FALSE;
    }

    /*  Get the array on which this span is present */
    arRef = MR_LdSpanArrayGet(ld, span, map); 

    pd = MR_ArPdGet(arRef, physArm, map);     // Get the Pd.

    if (pd != MR_PD_INVALID)
        *pDevHandle = MR_PdDevHandleGet(pd, map);  // Get dev handle from Pd.
    else {
        *pDevHandle = MR_PD_INVALID; // set dev handle as invalid.
        if ((raid->level >= 5) &&
            ((sc->device_id != MRSAS_INVADER) ||
            (sc->device_id == MRSAS_INVADER &&
             raid->regTypeReqOnRead != REGION_TYPE_UNUSED)))
             pRAID_Context->regLockFlags = REGION_TYPE_EXCLUSIVE;
        else if (raid->level == 1) {
            pd = MR_ArPdGet(arRef, physArm + 1, map); // Get Alternate Pd.
            if (pd != MR_PD_INVALID)
                *pDevHandle = MR_PdDevHandleGet(pd, map);//Get dev handle from Pd.
        }
    }

    *pdBlock += stripRef + MR_LdSpanPtrGet(ld, span, map)->startBlk;
    pRAID_Context->spanArm = (span << RAID_CTX_SPANARM_SPAN_SHIFT) | physArm;
    return retval;
}

/**
 * MR_GetSpanBlock     Calculates span block
 * Inputs:             LD 
 *                     row
 *                     PD span block
 *                     RAID map pointer
 * Outputs:            Span number
 *                     Error code 
 *
 * This routine calculates the span from the span block info.  
 */
u_int32_t MR_GetSpanBlock(u_int32_t ld, u_int64_t row, u_int64_t *span_blk, 
        MR_FW_RAID_MAP_ALL *map, int *div_error)
{
    MR_SPAN_BLOCK_INFO *pSpanBlock = MR_LdSpanInfoGet(ld, map);
    MR_QUAD_ELEMENT *quad;
    MR_LD_RAID *raid = MR_LdRaidGet(ld, map);
    u_int32_t span, j;
    u_int64_t blk, debugBlk;

    for (span=0; span < raid->spanDepth; span++, pSpanBlock++) {
        for (j=0; j < pSpanBlock->block_span_info.noElements; j++) {
            quad = &pSpanBlock->block_span_info.quad[j];
            if (quad->diff == 0) {
                *div_error = 1;
                return span;
            }
            if (quad->logStart <= row  &&  row <= quad->logEnd  &&  
                    (mega_mod64(row-quad->logStart, quad->diff)) == 0) {
                if (span_blk != NULL) {
                    blk =  mega_div64_32((row-quad->logStart), quad->diff);
                    debugBlk = blk;
                    blk = (blk + quad->offsetInSpan) << raid->stripeShift;
                    *span_blk = blk;
                }
                return span;
            }
        }
    }
    return span;
}

