#include "oled_ssd1306.h"

#include "i2c.h"
#include "cmsis_os.h"
#include <string.h>

#define OLED_ADDR_0        (0x3Cu << 1)
#define OLED_ADDR_1        (0x3Du << 1)
#define OLED_CMD_PREFIX    0x00u
#define OLED_DATA_PREFIX   0x40u
#define OLED_PAGE_COUNT    (OLED_SSD1306_HEIGHT / 8u)
#define OLED_BUFFER_SIZE   (OLED_SSD1306_WIDTH * OLED_PAGE_COUNT)
#define OLED_I2C_TIMEOUT   10u

static uint8_t oled_buffer[OLED_BUFFER_SIZE];
static uint16_t oled_address = OLED_ADDR_0;

static uint8_t oled_write_command(uint8_t command);
static uint8_t oled_write_data(const uint8_t *data, uint16_t length);
static const uint8_t *oled_get_font(char ch);

static const uint8_t font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t font_dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
static const uint8_t font_minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};

static const uint8_t font_digits[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
};

static const uint8_t font_letters[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
};

uint8_t oled_ssd1306_init(void)
{
    static const uint8_t init_commands[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF,
    };
    uint8_t i = 0;

    if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR_0, 2, OLED_I2C_TIMEOUT) == HAL_OK)
    {
        oled_address = OLED_ADDR_0;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR_1, 2, OLED_I2C_TIMEOUT) == HAL_OK)
    {
        oled_address = OLED_ADDR_1;
    }
    else
    {
        return 1;
    }

    osDelay(50);

    for (i = 0; i < sizeof(init_commands); i++)
    {
        if (oled_write_command(init_commands[i]) != 0)
        {
            return 1;
        }
    }

    oled_ssd1306_clear();
    oled_ssd1306_update();

    return 0;
}

void oled_ssd1306_clear(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

void oled_ssd1306_update(void)
{
    uint8_t page = 0;

    for (page = 0; page < OLED_PAGE_COUNT; page++)
    {
        oled_write_command((uint8_t)(0xB0u + page));
        oled_write_command(0x00);
        oled_write_command(0x10);
        oled_write_data(&oled_buffer[OLED_SSD1306_WIDTH * page], OLED_SSD1306_WIDTH);
    }
}

void oled_ssd1306_write_char(uint8_t x, uint8_t y, char ch)
{
    const uint8_t *font = oled_get_font(ch);
    uint8_t page = y / 8u;
    uint8_t i = 0;

    if (page >= OLED_PAGE_COUNT || x >= (OLED_SSD1306_WIDTH - 5u))
    {
        return;
    }

    for (i = 0; i < 5u; i++)
    {
        oled_buffer[page * OLED_SSD1306_WIDTH + x + i] = font[i];
    }

    if ((x + 5u) < OLED_SSD1306_WIDTH)
    {
        oled_buffer[page * OLED_SSD1306_WIDTH + x + 5u] = 0x00;
    }
}

void oled_ssd1306_write_string(uint8_t x, uint8_t y, const char *str)
{
    uint8_t cursor_x = x;
    uint8_t cursor_y = y;

    while (str != NULL && *str != '\0')
    {
        if (cursor_x > (OLED_SSD1306_WIDTH - 6u))
        {
            cursor_x = 0;
            cursor_y = (uint8_t)(cursor_y + 8u);
        }

        if (cursor_y >= OLED_SSD1306_HEIGHT)
        {
            return;
        }

        oled_ssd1306_write_char(cursor_x, cursor_y, *str);
        cursor_x = (uint8_t)(cursor_x + 6u);
        str++;
    }
}

static uint8_t oled_write_command(uint8_t command)
{
    uint8_t buffer[2] = {OLED_CMD_PREFIX, command};

    return (HAL_I2C_Master_Transmit(&hi2c2, oled_address, buffer, sizeof(buffer), OLED_I2C_TIMEOUT) == HAL_OK) ? 0u : 1u;
}

static uint8_t oled_write_data(const uint8_t *data, uint16_t length)
{
    uint8_t buffer[17];
    uint16_t offset = 0;
    uint16_t chunk = 0;

    while (offset < length)
    {
        chunk = (uint16_t)((length - offset) > 16u ? 16u : (length - offset));
        buffer[0] = OLED_DATA_PREFIX;
        memcpy(&buffer[1], &data[offset], chunk);

        if (HAL_I2C_Master_Transmit(&hi2c2, oled_address, buffer, (uint16_t)(chunk + 1u), OLED_I2C_TIMEOUT) != HAL_OK)
        {
            return 1;
        }

        offset = (uint16_t)(offset + chunk);
    }

    return 0;
}

static const uint8_t *oled_get_font(char ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return font_digits[ch - '0'];
    }

    if (ch >= 'a' && ch <= 'z')
    {
        ch = (char)(ch - 'a' + 'A');
    }

    if (ch >= 'A' && ch <= 'Z')
    {
        return font_letters[ch - 'A'];
    }

    if (ch == '.')
    {
        return font_dot;
    }

    if (ch == '-')
    {
        return font_minus;
    }

    return font_space;
}
