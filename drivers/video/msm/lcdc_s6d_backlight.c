/*
 * =====================================================================================
 *
 *       Filename:  lcdc_s6d_backlight.c
 *
 *    Description:  LCD Backlight control Function
 *
 *        Version:  1.0
 *        Created:  2010??11??25??15??21ë¶?38ì´?
 *       Revision:  1.0
 *       Compiler:  arm-linux-gcc
 *
 *         Author:  File (System S/W Group Kernel Part), 
 *        Company:  Samsung Electronics
 *
 * =====================================================================================
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "lcdc_s6d_backlight.h"
#include "msm_fb.h"

/* Panel Type */
#define PANEL_GP_TN			0x61ad10

extern unsigned int lcd_panel_type;
extern int lcd_type;
static int lcd_brightness = -1;

struct brt_value brt_table_ktd[] = {
		{ 255,	7 }, /* Max */
		{ 244,	9 },
		{ 233,	10 },
		{ 222,	11 },
		{ 211,	12 },
		{ 200,	13 },
		{ 189,	14 },
		{ 178,	16 },
		{ 166,	18 },
		{ 154,	20 }, /* default */
		{ 141,	22 },
		{ 127,	23 },
		{ 113,	24 },
		{ 99,	25 },
		{ 85,	26 },
		{ 71,	27 },
		{ 57,	28 },
		{ 43,	29 },
		{ 30,	31 }, /* Min */
		{ 20,	31 }, /* Dimming */
		{ 0,	32 }, /* Off */
};

#if defined(CONFIG_MACH_AMAZING_CDMA)
struct brt_value brt_table_aat[] = {
		{ 255,	14 }, /* Max 14 KTD  -330cd */
		{ 244,	15 },
		{ 233,	16 },
		{ 222,	17 },
		{ 211,	17 },
		{ 200,	18 },
		{ 189,	19 },
		{ 178,	20 },
		{ 166,	21 },
		{ 154,	22 }, /* Default 20 KTD - 190cd  */
		{ 141,	23 },
		{ 127,	24 },
		{ 113,	25 },
		{ 99,	26 },
		{ 85,	27 },
		{ 71,	28 },
		{ 57,	29 },
		{ 43,	30 },
		{ 30,	31 }, /* Min */
		{ 20,	31 }, /* Dimming */
		{ 0,	32 }, /* Off */
};

#else
struct brt_value brt_table_aat[] = {
		{ 255,	6 }, /* Max */
		{ 244,	7 },
		{ 233,	8 },
		{ 222,	9 },
		{ 211,	10 },
		{ 200,	11 },
		{ 189,	12 },
		{ 178,	13 },
		{ 166,	14 },
		{ 154,	15 }, /* default */
		{ 141,	16 },
		{ 127,	18 },
		{ 113,	20 },
		{ 99,	21 },
		{ 85,	23 },
		{ 71,	24 },
		{ 57,	26 },
		{ 43,	28 },
		{ 30,	30 }, /* Min */
		{ 20,	30 }, /* Dimming */
		{ 0,	32 }, /* Off */
};
#endif


#define MAX_BRT_STAGE_KTD (int)(sizeof(brt_table_ktd)/sizeof(struct brt_value))
#define MAX_BRT_STAGE_SHP (int)(sizeof(brt_table_shp)/sizeof(struct brt_value))
#define MAX_BRT_STAGE_AAT (int)(sizeof(brt_table_aat)/sizeof(struct brt_value))
static DEFINE_SPINLOCK(bl_ctrl_lock);

void lcdc_s6d_set_brightness_by_ktd259(int level)
{
	int pulse;
	int tune_level = 0;
	int i;
	int gpio_bl_ctrl;
	gpio_bl_ctrl = 32;

	spin_lock(&bl_ctrl_lock);
	if (level > 0) {
		if (level < MIN_BRIGHTNESS_VALUE) {
			tune_level = AAT_DIMMING_VALUE; /* DIMMING */
		} else {
			for (i = 0; i < MAX_BRT_STAGE_AAT - 1; i++) {
				if (level <= brt_table_aat[i].level
					&& level > brt_table_aat[i+1].level) {
					tune_level = brt_table_aat[i].tune_level;
					break;
				}
			}
		}
	} /*  BACKLIGHT is KTD model */

	if (!tune_level) {
		gpio_set_value(gpio_bl_ctrl, 0);
		mdelay(3);
		lcd_brightness = tune_level;
	} else {
		if (unlikely(lcd_brightness < 0)) {
			int val = gpio_get_value(gpio_bl_ctrl);
			if (val) {
				lcd_brightness = 0;
			gpio_set_value(gpio_bl_ctrl, 0);
			mdelay(3);
				printk(KERN_INFO "LCD Baklight init in boot time on kernel\n");
			}
		}
		if (!lcd_brightness) {
			gpio_set_value(gpio_bl_ctrl, 1);
			udelay(3);
			lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
		}

		pulse = (tune_level - lcd_brightness + MAX_BRIGHTNESS_IN_BLU)
						% MAX_BRIGHTNESS_IN_BLU;

		for (; pulse > 0; pulse--) {
			gpio_set_value(gpio_bl_ctrl, 0);
			udelay(3);
			gpio_set_value(gpio_bl_ctrl, 1);
			udelay(3);
		}

		lcd_brightness = tune_level;
	}
	mdelay(1);
	spin_unlock(&bl_ctrl_lock);
}


