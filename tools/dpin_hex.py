from argparse import ArgumentParser as parser
from subprocess import check_call as _cc
from json import load, dump
from os.path import dirname
from os import listdir


parse = parser(
	prog="pin to hex",
	description="convert an STM32 pin for a peripheral to encoded hex format"
)
flag =			{"action": "store_true"}
input_default =	{"action": "store", "type": int, "default": None}
CWD = dirname(__file__)
CPY = lambda x: _cc(f"echo {x}, | xclip -selection clipboard", shell=True)


def choose(items: list):
	if len(items) == 1: return items[0]
	pass  # TODO


if __name__ == "__main__":
	parse.add_argument('-cfg', action="store", type=str, default=None)
	
	parse.add_argument('-af', **input_default)
	parse.add_argument('-pnum', **input_default)
	
	parse.add_argument('-tim', **flag)
	parse.add_argument('-uart', **flag)
	parse.add_argument('-lpuart', **flag)
	parse.add_argument('-i2c', **flag)
	parse.add_argument('-spi', **flag)
	arg = parse.parse_args()
	arg.uart |= arg.lpuart
	
	if not arg.cfg:
		arg.cfg = choose([x for x in listdir(CWD) if x.endswith(".json")])
	
	with open(f"{CWD}/{arg.cfg}", "r") as f:
		cfg = load(f)
	
	
	while True:
		try:
			sub = 0
			if arg.tim:
				tim = input("tim: ") if not arg.pnum else arg.pnum
				try:    tim = cfg["tims"][f"TIM{int(tim)}"]
				except: tim = cfg["tims"][tim.upper()]
				clk, dev = tim
				clk = cfg["clocks"][clk]
				channel = max((int(input("channel: ")) - 1), 0)
				sub |= channel & 0x7
			elif arg.uart:
				uart = input(f"{'lp' if arg.lpuart else ''}uart: ") if not arg.pnum else arg.pnum
				try:    uart = cfg["uarts"][f"{'LP' if arg.lpuart else ''}UART{int(uart)}"]
				except: uart = cfg["uarts"][uart.upper()]
				clk, dev = uart
				clk = cfg["clocks"][clk]
			elif arg.i2c:
				i2c = input("i2c: ") if not arg.pnum else arg.pnum
				try:    i2c = cfg["i2cs"][f"I2C{int(i2c)}"]
				except: i2c = cfg["i2cs"][i2c.upper()]
				clk, dev = i2c
				clk = cfg["clocks"][clk]
			elif arg.spi:
				spi = input("spi: ") if not arg.pnum else arg.pnum
				try:    spi = cfg["spis"][f"SPI{int(spi)}"]
				except: spi = cfg["spis"][spi.upper()]
				clk, dev = spi
				clk = cfg["clocks"][clk]
			else:
				clk = input("clk: ")
				try:    clk = int(clk)
				except: clk = cfg["clocks"][clk.upper()]
				dev = int(input("offset: "), base=16) >> 10
			alt = int(input("alt: ")) if not arg.af else arg.af
			pin = input("pin: ")
			port = int(cfg["ports"][pin[0].upper()])
			pin = int(pin[1:])
			res = (
					((alt & 0xf) << 28)     |
					((pin & 0xf) << 24)     |
					((port & 0xf) << 20)    |
					((sub & 0xff) << 12)    |
					((dev & 0xff) << 4)		|
					((clk & 0xf) << 0)
			)
			#print(hex(sub))
			msg = f"{res:#0{10}x}UL".upper().replace("X", "x")
			CPY(msg); print(msg, end="\n\n")
		except KeyboardInterrupt:   exit(0)
		except Exception as e:      print(e); pass
