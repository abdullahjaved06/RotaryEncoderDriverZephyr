#include "rotary_encoder.h"
extern struct k_mutex menu_mutex;

uint8_t button_pressed = 0;
// Example mapping: adjust this to match your hardware configuration
const uint8_t BUTTON_UP = 2;     // GPIO pin 2
const uint8_t BUTTON_DOWN = 4;   // GPIO pin 4
const uint8_t BUTTON_LEFT = 3;   // GPIO pin 3
const uint8_t BUTTON_RIGHT = 5;  // GPIO pin 5
const uint8_t BUTTON_CENTER = 1; // GPIO pin 1

LOG_MODULE_REGISTER(RotaryEncoder);
static const struct i2c_dt_spec i2c_rot_enc_dev = I2C_DT_SPEC_GET(DT_NODELABEL(rotary_encoder));

// Function to write to a Seesaw register
static int seesaw_write_register(uint8_t regHigh, uint8_t regLow, const uint8_t *data, size_t length)
{
    uint8_t buf[length + 2];
    buf[0] = regHigh;
    buf[1] = regLow;
    memcpy(&buf[2], data, length);

    return i2c_write(i2c_rot_enc_dev.bus, buf, length + 2, i2c_rot_enc_dev.addr);
}

// Function to read from a Seesaw register
static int seesaw_read_register(uint8_t regHigh, uint8_t regLow, uint8_t *data, size_t length)
{
    uint8_t reg_buf[2] = {regHigh, regLow};

    // Write register address
    int ret = i2c_write(i2c_rot_enc_dev.bus, reg_buf, sizeof(reg_buf), i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to write register address");
        return ret;
    }

    // Read data from register
    ret = i2c_read(i2c_rot_enc_dev.bus, data, length, i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to read register");
    }

    return ret;
}
// Function to get encoder position
int32_t get_encoder_position(void)
{
    uint8_t buf[4];
    int ret = seesaw_read_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION, buf, sizeof(buf));
    if (ret < 0)
    {
        LOG_ERR("Failed to read encoder position");
        return ret;
    }

    return ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) |
           ((int32_t)buf[2] << 8) | (int32_t)buf[3];
}

// Function to get encoder delta
static int32_t get_encoder_delta(void)
{
    uint8_t buf[4];
    int ret = seesaw_read_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA, buf, sizeof(buf));
    if (ret < 0)
    {
        LOG_ERR("Failed to read encoder delta");
        return ret;
    }

    return ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) |
           ((int32_t)buf[2] << 8) | (int32_t)buf[3];
}

// // Function to enable encoder interrupts
// static int enable_encoder_interrupts(void)
// {
//     uint8_t data = 0x01;
//     int ret = seesaw_write_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET, &data, sizeof(data));
//     if (ret < 0)
//     {
//         LOG_ERR("Failed to enable encoder interrupts");
//     }
//     return ret;
// }

// Function to disable encoder interrupts
static int disable_encoder_interrupts(void)
{
    uint8_t data = 0x01;
    int ret = seesaw_write_register(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR, &data, sizeof(data));
    if (ret < 0)
    {
        LOG_ERR("Failed to disable encoder interrupts");
    }
    return ret;
}

int rot_encoder_init()
{
    if (!device_is_ready(i2c_rot_enc_dev.bus))
    {
        LOG_ERR("I2C bus not ready");
        return -ENODEV; // Return an appropriate error code
    }
    // Configure button pins as inputs
    uint32_t button_pins = (1 << BUTTON_UP_PIN) | (1 << BUTTON_DOWN_PIN) |
                           (1 << BUTTON_LEFT_PIN) | (1 << BUTTON_RIGHT_PIN) |
                           (1 << BUTTON_SELECT_PIN);
    if (configure_as_inputs(button_pins) < 0)
    {
        LOG_ERR("Failed to configure button pins as inputs");
        return -EIO;
    }
    // Enable pull-up resistors for button pins
    if (enable_pullups(button_pins) < 0)
    {
        LOG_ERR("Failed to enable pull-ups for button pins");
        return -EIO;
    }

    LOG_INF("Rotary encoder initialized successfully");
    return 0; // Return 0 on success
}

void read_rot_enc()
{
    int32_t last_position = get_encoder_position();
    if (last_position < 0)
    {
        LOG_ERR("Failed to read initial encoder position");
        return;
    }

    // Check position
    int32_t current_position = get_encoder_position();
    if (current_position >= 0 && current_position != last_position)
    {
        LOG_INF("Encoder position: %d", current_position);
        last_position = current_position;
    }

    uint32_t gpio_states;
    int ret;

    LOG_INF("Starting button state reader");

    /* Read GPIO states */
    ret = read_gpio_states(&gpio_states);
    if (ret == 0)
    {
        /* Log button states */
        log_button_states(gpio_states);
    }
    else
    {
        LOG_ERR("Failed to read GPIO states");
    }
}

/**
 * @brief Check if a specific button is pressed
 * @param pin The GPIO pin number of the button
 * @return True if the button is pressed, false otherwise
 */
bool is_button_pressed(uint8_t pin)
{
    uint8_t buf[4];
    if (seesaw_read_register(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, buf, sizeof(buf)) < 0)
    {
        LOG_ERR("Failed to read GPIO state");
        return false;
    }

    // Active-low logic for button states
    if (pin < 8)
    {
        return !(buf[0] & (1 << pin));
    }
    else if (pin < 16)
    {
        return !(buf[1] & (1 << (pin - 8)));
    }
    else if (pin < 24)
    {
        return !(buf[2] & (1 << (pin - 16)));
    }
    else
    {
        return !(buf[3] & (1 << (pin - 24)));
    }
}

/**
 * @brief Read the states of multiple buttons using a bitmask
 * @param pins A bitmask of the pins to check
 * @return Bitmask of button states (1 = pressed, 0 = not pressed)
 */
uint32_t read_button_states(uint32_t pins)
{
    uint8_t buf[4];
    uint8_t reg[2] = {SEESAW_GPIO_BASE, SEESAW_GPIO_BULK};

    // Write the register address to read GPIO states
    int ret = i2c_write(i2c_rot_enc_dev.bus, reg, sizeof(reg), i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to write to GPIO register");
        return 0;
    }

    // Read the GPIO input states
    ret = i2c_read(i2c_rot_enc_dev.bus, buf, sizeof(buf), i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to read GPIO state");
        return 0;
    }

    // Combine the 4 bytes into a 32-bit integer
    uint32_t state = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                     ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

    return state & pins; // Return only the states of the requested pins
}

static int enable_pullups(uint32_t pins)
{
    uint8_t buf[4] = {
        (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
        (uint8_t)(pins >> 8), (uint8_t)pins};
    return seesaw_write_register(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, buf, sizeof(buf));
}

static int configure_as_inputs(uint32_t pins)
{
    uint8_t buf[4] = {
        (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
        (uint8_t)(pins >> 8), (uint8_t)pins};
    return seesaw_write_register(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, buf, sizeof(buf));
}

/**
 * @brief Read GPIO states from Seesaw over I2C
 * @param[out] gpio_states Pointer to store the 32-bit GPIO state
 * @return 0 on success, negative error code on failure
 */
int read_gpio_states(uint32_t *gpio_states)
{
    uint8_t reg[2] = {SEESAW_GPIO_BASE, SEESAW_GPIO_BULK};
    uint8_t buf[4]; // Buffer to store the GPIO bulk response
    int ret;

    if (!device_is_ready(i2c_rot_enc_dev.bus))
    {
        LOG_ERR("I2C device is not ready");
        return -ENODEV;
    }

    /* Write the base and function register to initiate the read */
    ret = i2c_write(i2c_rot_enc_dev.bus, reg, sizeof(reg), SEESAW_I2C_ADDR);
    if (ret < 0)
    {
        LOG_ERR("Failed to write I2C register: %d", ret);
        return ret;
    }

    /* Read the 4-byte GPIO bulk state */
    ret = i2c_read(i2c_rot_enc_dev.bus, buf, sizeof(buf), SEESAW_I2C_ADDR);
    if (ret < 0)
    {
        LOG_ERR("Failed to read I2C data: %d", ret);
        return ret;
    }

    /* Combine the 4 bytes into a 32-bit integer */
    *gpio_states = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                   ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];

    return 0;
}

/**
 * @brief Log button states based on GPIO state
 * @param gpio_states 32-bit GPIO state read from Seesaw
 */
static void log_button_states(uint32_t gpio_states)
{

    if (!(gpio_states & (1 << BUTTON_UP)))
    {
        LOG_INF("UP button pressed");
    }
    if (!(gpio_states & (1 << BUTTON_DOWN)))
    {
        LOG_INF("DOWN button pressed");
    }
    if (!(gpio_states & (1 << BUTTON_LEFT)))
    {
        LOG_INF("LEFT button pressed");
    }
    if (!(gpio_states & (1 << BUTTON_RIGHT)))
    {
        LOG_INF("RIGHT button pressed");
    }
    if (!(gpio_states & (1 << BUTTON_CENTER)))
    {
        LOG_INF("CENTER button pressed");
    }
}

// /**
//  * @brief Enable rotary encoder interrupt for position changes.
//  *
//  * @param encoder Encoder index (default is 0).
//  * @return 0 on success, negative error code on failure.
//  */
// int enable_encoder_interrupt(uint8_t encoder) {
//     uint8_t cmd = 0x01;  // Enable interrupt
//     uint8_t reg = SEESAW_ENCODER_INTENSET + encoder;

//     // Write to the Seesaw encoder interrupt enable register
//     if (i2c_reg_write_byte(i2c_rot_enc_dev.bus, SEESAW_I2C_ADDR, reg, cmd)) {
//         printk("Failed to enable encoder interrupt.\n");
//         return -EIO;
//     }

//     printk("Encoder interrupt enabled for encoder %d.\n", encoder);
//     return 0;
// }
#define DEBOUNCE_TIME_MS 100 // adjust as needed

static int64_t last_interrupt_time = 0;
void interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int64_t now = k_uptime_get();
    if ((now - last_interrupt_time) > DEBOUNCE_TIME_MS)
    {
        last_interrupt_time = now;
        LOG_INF("Inerrupt Triggered.\n\r");

        k_sem_give(&menu_sem); // Signal the semaphore to start the menu operation
    }
}

void gpios_interrupt_init()
{
    int ret;
    /*INT button initialized as input pin*/
    ret = gpio_pin_configure_dt(&ROTARY_ENC_INTERRUPT_PIN, GPIO_INPUT);
    if (!device_is_ready(ROTARY_ENC_INTERRUPT_PIN.port))
    {
        printk("Error: gpio device %s is not ready\n", ROTARY_ENC_INTERRUPT_PIN.port->name);
        return;
    }
    if (ret != 0)
    {
        LOG_ERR("Error: failed to configure %s pin %d\n", ROTARY_ENC_INTERRUPT_PIN.port->name, ROTARY_ENC_INTERRUPT_PIN.pin);
        return;
    }

    /*Interrupt configured on Weight button pin*/
    ret = gpio_pin_interrupt_configure_dt(&ROTARY_ENC_INTERRUPT_PIN, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0)
    {
        printk("Error: failed to configure interrupt on %s pin %d\n", ROTARY_ENC_INTERRUPT_PIN.port->name, ROTARY_ENC_INTERRUPT_PIN.pin);
        return;
    }

    gpio_init_callback(&int_callback, interrupt_handler, BIT(ROTARY_ENC_INTERRUPT_PIN.pin));
    gpio_add_callback(ROTARY_ENC_INTERRUPT_PIN.port, &int_callback);
    // nrf_gpio_cfg_sense_set(ROTARY_ENC_INTERRUPT_PIN.pin, NRF_GPIO_PIN_SENSE_LOW);
}

// Function to write 1 byte to a specified Seesaw register
static int seesaw_write8(uint8_t regHigh, uint8_t regLow, uint8_t value)
{
    uint8_t buf[3] = {regHigh, regLow, value};

    // Write the buffer (address and value) to the I2C device
    int ret = i2c_write(i2c_rot_enc_dev.bus, buf, sizeof(buf), i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to write to Seesaw register (0x%02X, 0x%02X)", regHigh, regLow);
    }
    return ret;
}

// Function to enable the encoder interrupt
int enable_encoder_interrupt(uint8_t encoder)
{
    // Write 0x01 to the interrupt enable register for the specified encoder
    return seesaw_write8(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET + encoder, 0x01);
}

// Function to enable or disable GPIO interrupts on specified pins
static int enable_gpio_interrupts(uint32_t pins, bool enable)
{
    uint8_t cmd[6] = {SEESAW_GPIO_BASE, (enable ? SEESAW_GPIO_INTENSET : SEESAW_GPIO_INTENCLR),
                      (uint8_t)(pins >> 24), (uint8_t)(pins >> 16),
                      (uint8_t)(pins >> 8), (uint8_t)pins};

    int ret = i2c_write(i2c_rot_enc_dev.bus, cmd, sizeof(cmd), i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure GPIO interrupts");
        return ret;
    }

    return 0;
}

// Wrapper function to enable interrupts on all button pins
int enable_button_interrupts(bool enable)
{
    uint32_t button_pins = (1 << BUTTON_UP_PIN) | (1 << BUTTON_DOWN_PIN) |
                           (1 << BUTTON_LEFT_PIN) | (1 << BUTTON_RIGHT_PIN) |
                           (1 << BUTTON_SELECT_PIN);

    return enable_gpio_interrupts(button_pins, enable);
}

/*functions for clearing interrupt*/

// Function to clear GPIO interrupt flags
static int clear_gpio_interrupt_flags()
{
    uint8_t cmd[2] = {SEESAW_GPIO_BASE, SEESAW_GPIO_INTFLAG};
    uint8_t resp[4] = {0};

    // Read the interrupt flags
    int ret = i2c_write_read(i2c_rot_enc_dev.bus, i2c_rot_enc_dev.addr, cmd, sizeof(cmd), resp, sizeof(resp));
    if (ret < 0)
    {
        LOG_ERR("Failed to read GPIO interrupt flags");
        return ret;
    }

    // Log which pins triggered the interrupt
    uint32_t flags = (resp[0] << 24) | (resp[1] << 16) | (resp[2] << 8) | resp[3];
    // LOG_INF("GPIO Interrupt flags: 0x%08X", flags);

    return 0;
}

// Function to clear encoder interrupt
static int clear_encoder_interrupt_flags(uint8_t encoder)
{
    uint8_t cmd[3] = {SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR + encoder, 0x01};

    // Write 0x01 to clear the encoder interrupt
    int ret = i2c_write(i2c_rot_enc_dev.bus, cmd, sizeof(cmd), i2c_rot_enc_dev.addr);
    if (ret < 0)
    {
        LOG_ERR("Failed to clear encoder interrupt flags");
        return ret;
    }

    LOG_INF("Encoder interrupt flags cleared.");
    return 0;
}

void clear_Buttons_INT()
{
    // Clear GPIO interrupt flags
    clear_gpio_interrupt_flags();
}

void clear_Encoder_INT()
{
    // Clear encoder interrupt flags if needed
    clear_encoder_interrupt_flags(0); // Assuming encoder 0
}