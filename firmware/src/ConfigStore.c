/*
 * Single-page flash persistence for the config record. See ConfigStore.h.
 */
#include <string.h>
#include <stddef.h>
#include "ConfigStore.h"
#include "main.h"
#include "Output.h"   /* TORQUE_FULL_SCALE for the default clamp values */

/* Last 256B page of L152RE's 512K flash (0x08000000 + 0x80000 -
 * FLASH_PAGE_SIZE) -- L1's page size is 256B, not F103's 1KB. L1 program
 * flash only supports word (32-bit) writes. */
#define CONFIG_FLASH_PAGE_ADDR 0x0807FF00U
#define CONFIG_FLASH_MAGIC     0x54525153U   /* "TRQS" */

/* Record layout version. Starts at 1 (never 0 or 0xFF -- those collide with
 * zeroed/erased flash). No migration: an old-version record is rejected and
 * defaults apply -- see AGENTS.md. Bump this on any field-layout change. */
#define CONFIG_FLASH_VERSION   1U

/* Fixed, padded record size -- a new field consumes reserved padding in
 * place, so this never needs to grow for a small addition. */
#define CONFIG_FLASH_RECORD_SIZE 104U

typedef struct {
  uint32_t magic;
  uint32_t crc;      /* CRC32 over everything after magic+crc */
  uint8_t  version;
  uint8_t  servoMode;
  uint8_t  followingErrorHardFault;
  uint8_t  polarityLearned;
  int32_t  posGainQ8, velGainKpQ8, velGainKiQ8;
  int32_t  torqueClampMin, torqueClampMax, velocityClamp;
  int32_t  followingErrorLimitCounts;
  int8_t   polaritySign;
  uint8_t  enabled;
  uint8_t  _pad[2];
  int32_t  zeroTrimCounts;
  int32_t  analogVelocityScaleQ8;
  int32_t  analogVelocityHysteresisCounts;
  int32_t  sustainedCurrentMinTorque;
  int32_t  sustainedCurrentWindowTicks;
  int32_t  rampAccelCountsPerTickPerSec;
  int32_t  commandVelocityClamp;
  int32_t  velocityFilterShift;
  int32_t  accelerationFeedforwardQ8;
  int32_t  velocityJerk;
  int32_t  positionMoveFeedforwardQ8;
  int32_t  velocityErrorDeadbandCounts;
  uint8_t  _reserved[12];  /* pads to CONFIG_FLASH_RECORD_SIZE -- zero-filled by
                               ConfigStoreSave()'s memset before any field is set, so
                               these read back as 0 until a future field claims some of
                               them (shrink this array by exactly as many bytes as the
                               new field adds; the _Static_assert below catches a
                               mismatch). */
} ConfigFlashRecord;

_Static_assert(sizeof(ConfigFlashRecord) == CONFIG_FLASH_RECORD_SIZE,
               "ConfigFlashRecord must stay exactly CONFIG_FLASH_RECORD_SIZE -- shrink "
               "_reserved by however many bytes a new field adds, don't just let the "
               "struct grow past it");
_Static_assert(CONFIG_FLASH_RECORD_SIZE <= 256U, "config must fit the L152 flash page");
_Static_assert(CONFIG_FLASH_RECORD_SIZE % 4U == 0U,
               "CONFIG_FLASH_RECORD_SIZE must stay a multiple of 4 bytes -- L152's word "
               "program path below writes exactly sizeof(rec)/4 words");

/* CRC32 (IEEE 802.3, reflected, poly 0xEDB88320) -- table-less. */
static uint32_t Crc32(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFU;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++)
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

static uint32_t RecordCrc(const ConfigFlashRecord *r) {
  return Crc32((const uint8_t *)r + 8U, (uint32_t)sizeof(*r) - 8U);
}

void ConfigDataSetDefaults(ConfigData *cfg) {
  memset(cfg, 0, sizeof(*cfg));
  cfg->positionMoveFeedforwardQ8 = 0;
  cfg->servoMode = 0;                 /* Mode A (step/dir) */
  cfg->enabled = 0;                   /* don't auto-start on a fresh/defaulted config --
                                          see AppSetEnabled()/main()'s boot sequence */
  cfg->followingErrorHardFault = 0;   /* soft warning by default -- see Fault.h */
  /* Deliberately weak, not unity and not near-zero -- see CONFIG_DEFAULT_*_Q8's
   * comment in ConfigStore.h. DO NOT treat these as tuned -- increase/decrease
   * on the real bench while watching for oscillation (too high) or an
   * unresponsive/dead-feeling axis (too low). */
  cfg->posGainQ8 = CONFIG_DEFAULT_POS_GAIN_Q8;
  cfg->velGainKpQ8 = CONFIG_DEFAULT_VEL_GAIN_KP_Q8;
  cfg->velGainKiQ8 = CONFIG_DEFAULT_VEL_GAIN_KI_Q8;
  cfg->torqueClampMin = -TORQUE_FULL_SCALE;
  cfg->torqueClampMax = TORQUE_FULL_SCALE;
  cfg->velocityClamp = 10000;
  cfg->followingErrorLimitCounts = 0; /* 0 == disabled until configured */
  cfg->polarityLearned = 0;
  cfg->polaritySign = 1;
  cfg->zeroTrimCounts = 0;            /* just defaulting to zero -- tune on the bench */
  cfg->analogVelocityScaleQ8 = CONFIG_DEFAULT_ANALOG_SCALE_Q8;
  cfg->analogVelocityHysteresisCounts = CONFIG_DEFAULT_ANALOG_HYSTERESIS_COUNTS;
  cfg->sustainedCurrentMinTorque = TORQUE_FULL_SCALE / 10;   /* 10% -- see Fault.h */
  cfg->sustainedCurrentWindowTicks = CONFIG_DEFAULT_SUSTAINED_CURRENT_WINDOW_TICKS;
  cfg->rampAccelCountsPerTickPerSec = CONFIG_DEFAULT_RAMP_ACCEL;
  cfg->velocityErrorDeadbandCounts = CONFIG_DEFAULT_VELOCITY_ERROR_DEADBAND_COUNTS;
}

int ConfigStoreSave(const ConfigData *cfg) {
  ConfigFlashRecord rec;
  memset(&rec, 0, sizeof(rec));
  rec.servoMode = cfg->servoMode;
  rec.enabled = cfg->enabled;
  rec.followingErrorHardFault = cfg->followingErrorHardFault;
  rec.polarityLearned = cfg->polarityLearned;
  rec.posGainQ8 = cfg->posGainQ8;
  rec.velGainKpQ8 = cfg->velGainKpQ8;
  rec.velGainKiQ8 = cfg->velGainKiQ8;
  rec.torqueClampMin = cfg->torqueClampMin;
  rec.torqueClampMax = cfg->torqueClampMax;
  rec.velocityClamp = cfg->velocityClamp;
  rec.commandVelocityClamp = cfg->commandVelocityClamp;
  rec.velocityFilterShift = cfg->velocityFilterShift;
  rec.accelerationFeedforwardQ8 = cfg->accelerationFeedforwardQ8;
  rec.velocityJerk = cfg->velocityJerk;
  rec.positionMoveFeedforwardQ8 = cfg->positionMoveFeedforwardQ8;
  rec.followingErrorLimitCounts = cfg->followingErrorLimitCounts;
  rec.polaritySign = cfg->polaritySign;
  rec.zeroTrimCounts = cfg->zeroTrimCounts;
  rec.analogVelocityScaleQ8 = cfg->analogVelocityScaleQ8;
  rec.analogVelocityHysteresisCounts = cfg->analogVelocityHysteresisCounts;
  rec.sustainedCurrentMinTorque = cfg->sustainedCurrentMinTorque;
  rec.sustainedCurrentWindowTicks = cfg->sustainedCurrentWindowTicks;
  rec.rampAccelCountsPerTickPerSec = cfg->rampAccelCountsPerTickPerSec;
  rec.velocityErrorDeadbandCounts = cfg->velocityErrorDeadbandCounts;
  rec.version = CONFIG_FLASH_VERSION;
  rec.magic = CONFIG_FLASH_MAGIC;
  rec.crc = RecordCrc(&rec);

  int ok = 1;
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = CONFIG_FLASH_PAGE_ADDR;
  erase.NbPages     = 1U;
  uint32_t pageError = 0U;
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) ok = 0;

  if (ok) {
    /* L1 program-flash only supports FLASH_TYPEPROGRAM_WORD (32-bit).
     * ConfigFlashRecord's size is kept a multiple of 4 by construction. */
    const uint32_t *src = (const uint32_t *)&rec;
    uint32_t nWords = (uint32_t)(sizeof(rec) / 4U);
    for (uint32_t i = 0; i < nWords; i++) {
      uint32_t addr = CONFIG_FLASH_PAGE_ADDR + i * 4U;
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) { ok = 0; break; }
      if (*(volatile uint32_t *)(uintptr_t)addr != src[i]) { ok = 0; break; }
    }
  }

  HAL_FLASH_Lock();
  return ok ? 0 : -1;
}

int ConfigStoreLoad(ConfigData *out) {
  const ConfigFlashRecord *rec = (const ConfigFlashRecord *)(uintptr_t)CONFIG_FLASH_PAGE_ADDR;
  if (rec->magic != CONFIG_FLASH_MAGIC) return -1;
  if (rec->version != CONFIG_FLASH_VERSION) return -1;   /* no migration -- see ConfigStore.h */
  if (rec->crc != Crc32((const uint8_t *)rec + 8U, CONFIG_FLASH_RECORD_SIZE - 8U)) return -1;

  out->servoMode = rec->servoMode;
  out->enabled = rec->enabled;
  out->followingErrorHardFault = rec->followingErrorHardFault;
  out->posGainQ8 = rec->posGainQ8;
  out->velGainKpQ8 = rec->velGainKpQ8;
  out->velGainKiQ8 = rec->velGainKiQ8;
  out->torqueClampMin = rec->torqueClampMin;
  out->torqueClampMax = rec->torqueClampMax;
  out->velocityClamp = rec->velocityClamp;
  out->commandVelocityClamp = rec->commandVelocityClamp;
  out->velocityFilterShift = rec->velocityFilterShift;
  out->accelerationFeedforwardQ8 = rec->accelerationFeedforwardQ8;
  out->velocityJerk = rec->velocityJerk;
  out->positionMoveFeedforwardQ8 = rec->positionMoveFeedforwardQ8;
  out->followingErrorLimitCounts = rec->followingErrorLimitCounts;
  out->polarityLearned = rec->polarityLearned;
  out->polaritySign = rec->polaritySign;
  out->zeroTrimCounts = rec->zeroTrimCounts;
  out->analogVelocityScaleQ8 = rec->analogVelocityScaleQ8;
  out->analogVelocityHysteresisCounts = rec->analogVelocityHysteresisCounts;
  out->sustainedCurrentMinTorque = rec->sustainedCurrentMinTorque;
  out->sustainedCurrentWindowTicks = rec->sustainedCurrentWindowTicks;
  out->rampAccelCountsPerTickPerSec = rec->rampAccelCountsPerTickPerSec;
  out->velocityErrorDeadbandCounts = rec->velocityErrorDeadbandCounts;
  return 0;
}

void ConfigStoreErase(void) {
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = CONFIG_FLASH_PAGE_ADDR;
  erase.NbPages     = 1U;
  uint32_t pageError = 0U;
  HAL_FLASHEx_Erase(&erase, &pageError);
  HAL_FLASH_Lock();
}
