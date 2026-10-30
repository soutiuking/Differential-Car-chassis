/**
  ******************************************************************************
  * @file    ssd1306.c
  * @brief   128×64 SSD1306 显存与分页刷新。
  *
  * 每页 128 字节，通过 I2C1 以 0x40 数据流发送；nav_task 每 20 ms 刷一页，
  * 避免一次整屏阻塞电机控制。
  ******************************************************************************
  */
#include "ssd1306.h"
#include "ssd1306_font.h"
#include "i2c.h"
#include "main.h"

#define SSD1306_ADDR          (0x3CU << 1)
#define SSD1306_TIMEOUT_MS    20U

static uint8_t s_fb[8][128];
static uint8_t s_page_buf[129];

static bool ssd1306_cmd(const uint8_t *cmd, uint16_t len)
{
    return HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR,
                                   (uint8_t *)cmd, len,
                                   SSD1306_TIMEOUT_MS) == HAL_OK;
}

bool ssd1306_init(void)
{
    static const uint8_t init_cmds[] =
    {
        0x00,
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0x2E,
        0xAF
    };
    uint8_t page;

    if (!ssd1306_cmd(init_cmds, sizeof(init_cmds)))
    {
        return false;
    }

    ssd1306_clear();
    for (page = 0U; page < 8U; page++)
    {
        (void)ssd1306_flush_page(page);
    }
    return true;
}

void ssd1306_clear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0U; page < 8U; page++)
    {
        for (x = 0U; x < 128U; x++)
        {
            s_fb[page][x] = 0U;
        }
    }
}

void ssd1306_clear_page(uint8_t page)
{
    uint8_t x;

    if (page >= 8U)
    {
        return;
    }
    for (x = 0U; x < 128U; x++)
    {
        s_fb[page][x] = 0U;
    }
}

void ssd1306_draw_text(uint8_t x, uint8_t page, const char *text)
{
    uint8_t pos = x;
    char c;

    if ((text == NULL) || (page >= 8U))
    {
        return;
    }

    while ((*text != '\0') && (pos < 123U))
    {
        c = *text++;
        if ((c >= 'a') && (c <= 'z'))
        {
            c = (char)(c - 'a' + 'A');
        }
        if ((c < (char)SSD1306_FONT_FIRST) || (c > (char)SSD1306_FONT_LAST))
        {
            c = '?';
        }
        {
            const uint8_t *glyph = s_ssd1306_font[(uint8_t)c - SSD1306_FONT_FIRST];
            uint8_t col;
            uint8_t row;

            for (col = 0U; col < 5U; col++)
            {
                for (row = 0U; row < 7U; row++)
                {
                    if ((glyph[col] & (1U << row)) != 0U)
                    {
                        s_fb[page][pos + col] |= (uint8_t)(1U << row);
                    }
                }
            }
        }
        pos = (uint8_t)(pos + 6U);
    }
}

void ssd1306_draw_i32(uint8_t x, uint8_t page, int32_t value, uint8_t width)
{
    char digits[12];
    uint8_t count = 0U;
    uint8_t total;
    uint8_t pad;
    uint8_t i;
    uint32_t mag;
    bool negative = (value < 0);

    if (value == 0)
    {
        digits[count++] = '0';
    }
    else
    {
        mag = negative ? (uint32_t)(-(value + 1)) + 1U : (uint32_t)value;
        while (mag > 0U)
        {
            digits[count++] = (char)('0' + (mag % 10U));
            mag /= 10U;
        }
    }

    total = (uint8_t)(count + (negative ? 1U : 0U));
    pad = (width > total) ? (uint8_t)(width - total) : 0U;

    for (i = 0U; i < pad; i++)
    {
        char one[2] = { ' ', '\0' };
        ssd1306_draw_text((uint8_t)(x + (i * 6U)), page, one);
    }

    if (negative)
    {
        char one[2] = { '-', '\0' };
        ssd1306_draw_text((uint8_t)(x + (pad * 6U)), page, one);
        pad++;
    }

    for (i = 0U; i < count; i++)
    {
        char one[2];
        one[0] = digits[count - 1U - i];
        one[1] = '\0';
        ssd1306_draw_text((uint8_t)(x + ((pad + i) * 6U)), page, one);
    }
}

bool ssd1306_flush_page(uint8_t page)
{
    uint8_t cmd[4];

    if (page >= 8U)
    {
        return false;
    }

    cmd[0] = 0x00;
    cmd[1] = (uint8_t)(0xB0U | page);
    cmd[2] = 0x00;
    cmd[3] = 0x10;
    if (!ssd1306_cmd(cmd, sizeof(cmd)))
    {
        return false;
    }

    s_page_buf[0] = 0x40;
    {
        uint8_t i;
        for (i = 0U; i < 128U; i++)
        {
            s_page_buf[1U + i] = s_fb[page][i];
        }
    }

    return HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR,
                                   s_page_buf, sizeof(s_page_buf),
                                   SSD1306_TIMEOUT_MS) == HAL_OK;
}
