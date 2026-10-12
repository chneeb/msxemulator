#include "bsp/board.h"
#include "tusb.h"
#include <stdbool.h>
#include "hidparser/hidparser.h"

#define INVALID_REPORT_ID -1
// means 1/X of half range of analog would be dead zone
#define DEAD_ZONE 4U

#define CONFIG_BUTTON_A     GAMEPAD_BUTTON_0
#define CONFIG_BUTTON_B     GAMEPAD_BUTTON_1
// Select and Start on generic pads (button usages). Layouts differ, so two
// usages each, as in uf2loader's menu.
#define CONFIG_BUTTON_SELECT_A  7
#define CONFIG_BUTTON_SELECT_B  9
#define CONFIG_BUTTON_START_A   8
#define CONFIG_BUTTON_START_B   10

static const char *const BUTTON_NAMES[] = {"NONE", "UP", "RIGHT", "DOWN", "LEFT", "A", "B"};
//(hat format, 8 is released, 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW)
static const uint8_t HAT_SWITCH_TO_DIRECTION_BUTTONS[] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001, 0b0000};

volatile uint8_t gamepad_info;      // Host gamepad info

typedef union
{
	struct
	{
		bool up : 1;
		bool right : 1;
		bool down : 1;
		bool left : 1;
		bool button1 : 1;
		bool button2 : 1;
		bool start : 1;
		bool select : 1;
	};
	struct
	{
		uint8_t all_direction : 4;
		uint8_t all_buttons : 2;
	};
	uint8_t value : 8;
} pad_buttons;

// Known pads, decoded from fixed byte positions instead of the report
// descriptor: cheap SNES clones don't describe themselves reliably (0079:0011
// sends 01 7f 7f XX YY ..., and the parser reads the constant 01 as X, so Left
// is held). Maps measured for frank-snes (drivers/usbhid/hid_app.c), as
// adopted by galagino_pizero/usb_input.c; 0810:e501 from pico-infonesPlus.
typedef struct { uint8_t byte, mask; } pad_bit_t;
typedef struct pad_map
{
	uint16_t vid, pid;
	bool hat;          // false: axes at dx/dy (0x7f centre), true: hat in low nibble of dx
	uint8_t dx, dy;
	pad_bit_t a, b;    // A -> TRIG A, B -> TRIG B
	pad_bit_t start, select;  // Select + Start: menu (Start/Select bits from uf2loader)
} pad_map_t;

#define SNES_CLONE_BUTTONS {5, 0x20}, {5, 0x40}, {6, 0x20}, {6, 0x10}

static const pad_map_t pad_maps[] = {
	{0x0079, 0x0006, false, 0, 1, SNES_CLONE_BUTTONS},   // DragonRise "USB Gamepad"
	{0x0079, 0x0011, false, 3, 4, SNES_CLONE_BUTTONS},   // SNES clone
	{0x081f, 0xe401, false, 0, 1, SNES_CLONE_BUTTONS},   // SNES clone
	{0x0810, 0xe501, false, 3, 4, SNES_CLONE_BUTTONS},   // SNES clone variant
	{0x046d, 0xc219, true,  5, 0, SNES_CLONE_BUTTONS},   // Logitech
	{0x11ff, 0x3331, false, 0, 1, {5, 0x80}, {5, 0x40}, {6, 0x20}, {6, 0x10}},
	{0x2563, 0x0575, true,  2, 0, {0, 0x04}, {0, 0x02}, {1, 0x02}, {1, 0x01}},
	{0xfeed, 0x2320, true,  5, 0, {6, 0x01}, {6, 0x02}, {7, 0x08}, {7, 0x04}},
};

const pad_map_t *joystick_find_map(uint16_t vid, uint16_t pid)
{
	for (unsigned i = 0; i < sizeof(pad_maps) / sizeof(pad_maps[0]); i++)
		if (pad_maps[i].vid == vid && pad_maps[i].pid == pid)
			return &pad_maps[i];
	return NULL;
}

static inline bool pad_bit(const uint8_t *r, uint16_t len, pad_bit_t b)
{
	return b.byte < len && (r[b.byte] & b.mask);
}

extern uint32_t menumode;
extern volatile uint8_t keypressed;

// Play without a keyboard: Select + Start does what F12 does (opens the menu,
// or leaves it), and in the menu the pad sends the keys the menu reads:
// Up/Down = cursor keys, Left/Right = Page Up/Down, A = Enter, B = Esc.
static void pad_menu_input(pad_buttons current)
{
	static pad_buttons prev = {0};
	uint8_t pressed = current.value & ~prev.value;
	bool combo = current.start && current.select;
	bool combo_new = combo && !(prev.start && prev.select);
	prev = current;

	if (combo_new) {
		if (menumode == 0) {
			menumode = 1;
			keypressed = 0;
		} else {
			keypressed = 0x45;   // F12
		}
		return;
	}
	if (menumode == 0 || combo)
		return;
	pad_buttons p;
	p.value = pressed;
	if (p.up)      keypressed = 0x52;
	if (p.down)    keypressed = 0x51;
	if (p.left)    keypressed = 0x4b;   // Page Up
	if (p.right)   keypressed = 0x4e;   // Page Down
	if (p.button1) keypressed = 0x28;   // Enter
	if (p.button2) keypressed = 0x29;   // Esc
}

void parse_mapped_report(const pad_map_t *m, uint8_t const *r, uint16_t len)
{
	pad_buttons current = {0};
	current.value = 0;
	if (!m->hat)
	{
		if (m->dx < len && r[m->dx] < 0x40) current.left = 1;
		if (m->dx < len && r[m->dx] > 0xc0) current.right = 1;
		if (m->dy < len && r[m->dy] < 0x40) current.up = 1;
		if (m->dy < len && r[m->dy] > 0xc0) current.down = 1;
	}
	else if (m->dx < len)
	{
		current.all_direction = HAT_SWITCH_TO_DIRECTION_BUTTONS[(r[m->dx] & 0x0f) < 8 ? (r[m->dx] & 0x0f) : 8];
	}
	if (pad_bit(r, len, m->a)) current.button1 = 1;
	if (pad_bit(r, len, m->b)) current.button2 = 1;
	if (pad_bit(r, len, m->start)) current.start = 1;
	if (pad_bit(r, len, m->select)) current.select = 1;
	pad_menu_input(current);

	uint8_t value = current.value;
	uint8_t info = 0x3f;
	if (value & 32) info &= 0x1f;   // TRIG B
	if (value & 16) info &= 0x2f;   // TRIG A
	if (value & 8)  info &= 0x3b;   // Left
	if (value & 4)  info &= 0x3d;   // Down
	if (value & 2)  info &= 0x37;   // Right
	if (value & 1)  info &= 0x3e;   // Up
	gamepad_info = info;
}

HID_ReportInfo_t *my_hid_info[4] = {NULL,NULL,NULL,NULL};
int16_t reportID;
pad_buttons previous = {0};
uint8_t g_dev_addr = 255;
uint8_t g_instance = 0;

//called from parser for filtering report items
bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t *const CurrentItem)
{
	if (CurrentItem->ItemType != HID_REPORT_ITEM_In)
		return false;

	if (reportID == INVALID_REPORT_ID)
	{
		reportID = CurrentItem->ReportID;
	}
	switch (CurrentItem->Attributes.Usage.Page)
	{
		case HID_USAGE_PAGE_DESKTOP:
			switch (CurrentItem->Attributes.Usage.Usage)
			{
				case HID_USAGE_DESKTOP_X:
				case HID_USAGE_DESKTOP_Y:
				case HID_USAGE_DESKTOP_HAT_SWITCH:
				case HID_USAGE_DESKTOP_DPAD_UP:
				case HID_USAGE_DESKTOP_DPAD_DOWN:
				case HID_USAGE_DESKTOP_DPAD_LEFT:
				case HID_USAGE_DESKTOP_DPAD_RIGHT:
					return true;
			}
			return false;
		case HID_USAGE_PAGE_BUTTON:
			return true;
	}
	return false;
}

static inline bool USB_GetHIDReportItemInfoWithReportId(const uint8_t *ReportData, HID_ReportItem_t *const ReportItem)
{
	if (ReportItem->ReportID)
	{
		if (ReportItem->ReportID != ReportData[0])
			return false;

		ReportData++;
	}
	return USB_GetHIDReportItemInfo(ReportItem->ReportID, ReportData, ReportItem);
}

void parse_gamepad_report(uint8_t const *report, uint16_t len , uint8_t instance)
{
	pad_buttons current = {0};
	current.value = 0;
	HID_ReportItem_t *item = my_hid_info[instance]->FirstReportItem;  // MUST CHANGE to instance id 
	//iterate filtered reports info to match report from data
	while (item)
	{
		if (USB_GetHIDReportItemInfoWithReportId(report, item))
		{
			switch (item->Attributes.Usage.Page)
			{
			case HID_USAGE_PAGE_DESKTOP:
				switch (item->Attributes.Usage.Usage)
				{
				case HID_USAGE_DESKTOP_X:
				{
					uint32_t range_half = (item->Attributes.Logical.Maximum - item->Attributes.Logical.Minimum) / 2;
					uint32_t dead_zone_range = range_half / DEAD_ZONE;
					if (item->Value < (range_half - dead_zone_range))
					{
						current.left |= 1;
					}
					else if (item->Value > (range_half + dead_zone_range))
					{
						current.right |= 1;
					}
				}
				break;
				case HID_USAGE_DESKTOP_Y:
				{
					uint32_t range_half = (item->Attributes.Logical.Maximum - item->Attributes.Logical.Minimum) / 2;
					uint32_t dead_zone_range = range_half / DEAD_ZONE;
					if (item->Value < (range_half - dead_zone_range))
					{
						current.up |= 1;
					}
					else if (item->Value > (range_half + dead_zone_range))
					{
						current.down |= 1;
					}
				}
				break;
				case HID_USAGE_DESKTOP_HAT_SWITCH:
					current.all_direction |= HAT_SWITCH_TO_DIRECTION_BUTTONS[item->Value];
					break;
				case HID_USAGE_DESKTOP_DPAD_UP:
					current.up |= 1;
					break;
				case HID_USAGE_DESKTOP_DPAD_RIGHT:
					current.right |= 1;
					break;
				case HID_USAGE_DESKTOP_DPAD_DOWN:
					current.down |= 1;
					break;
				case HID_USAGE_DESKTOP_DPAD_LEFT:
					current.left |= 1;
					break;
				}
				break;
			case HID_USAGE_PAGE_BUTTON:
			{
				uint8_t usage = item->Attributes.Usage.Usage;
				if (usage == CONFIG_BUTTON_A)
				{
					if (item->Value)
					{
						current.button1 = 1;
					}
				}
				else if (usage == CONFIG_BUTTON_B)
				{
					if (item->Value)
					{
						current.button2 = 1;
					}
				}
				else if (usage == CONFIG_BUTTON_SELECT_A || usage == CONFIG_BUTTON_SELECT_B)
				{
					if (item->Value)
					{
						current.select = 1;
					}
				}
				else if (usage == CONFIG_BUTTON_START_A || usage == CONFIG_BUTTON_START_B)
				{
					if (item->Value)
					{
						current.start = 1;
					}
				}
			}
			break;
			}
		}
		item = item->Next;
	}
	if (previous.value != current.value)
	{
		//of course you can use GPIO here for hiven buttons
		uint8_t value = current.value;

        gamepad_info=0x3f;

        if(value&32) {gamepad_info&=0x1f;};   // TRIG B
        if(value&16) {gamepad_info&=0x2f;};   // TRIG A
        if(value&8) {gamepad_info&=0x3b;};   // Left
        if(value&4) {gamepad_info&=0x3d;};   // Down
        if(value&2) {gamepad_info&=0x37;};  // Right
        if(value&1) {gamepad_info&=0x3e;};  // Up

		// if (!value)
		// {
		// 	printf(BUTTON_NAMES[0]);
		// }
		// else
		// {
		// 	bool first = true;
		// 	for (int i = 1; i <= 6; i++)
		// 	{
		// 		if (value & 1)
		// 		{
		// 			if (first)
		// 			{
		// 				printf("%s", BUTTON_NAMES[i]);
		// 				first = false;
		// 			}
		// 			else
		// 			{
		// 				printf(", %s", BUTTON_NAMES[i]);
		// 			}
		// 		}
		// 		value >>= 1;
		// 	}
		// }
		// printf("\n");
		pad_menu_input(current);
		previous.value = current.value;
	}
}

