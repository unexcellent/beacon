#include <stdint.h>
#include "soc/isp_struct.h"

/* Patch ISP hardware registers for RAW10 bypass after esp_isp_new_processor().
 *
 * esp_isp_new_processor() allocates the ISP clock and interrupt but sets
 * frame_cfg based on the placeholder color types passed in the config struct.
 * For a straight-through RAW10 path we need to override those fields directly:
 *
 *   hadr_num = ceil(h_res * 10 / 32) - 1   (32-bit words per line, minus 1)
 *   vadr_num = v_res - 1                    (lines per frame, minus 1)
 *   isp_en   = 0                            (processing disabled = passthrough)
 *
 * Without this the ISP hardware never asserts frame-done and the CSI DMA
 * transfer-complete callback never fires. */
void isp_bypass_raw10_patch(uint32_t h_res, uint32_t v_res)
{
    ISP.frame_cfg.hadr_num = ((h_res * 10u + 31u) / 32u) - 1u;
    ISP.frame_cfg.vadr_num = v_res - 1u;
    ISP.cntl.isp_en        = 0;
}
