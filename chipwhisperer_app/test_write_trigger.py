import chipwhisperer as cw
import time
import logging
import sys
import argparse
import matplotlib.pyplot as plt

from struct import *
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)

check = False
in_list = True

address_list = [0x0005f300, 0x00071400, 0x00078400]

with_plt = check

def reboot_flush(scope, target):
    scope.io.nrst = False
    time.sleep(0.05)
    scope.io.nrst = "high_z"
    time.sleep(0.05)
    #Flush garbage too
    target.flush()


def act(scope, target, addr, size):
    data = pack("<II", addr, size)
    ret = target.simpleserial_write('b', data)


def glitch_main(scope, target):
    gc = cw.GlitchController(groups=["success", "reset", "normal"], parameters=["width", "offset", "ext_offset", "address", "tries"])
    #gc.display_stats()

    max_successes = 3
    if check:
        num_tries = 50
    else:
        num_tries = 10 if not in_list else 50

    total_successes = 0
    successes = 0
    resets = 0
    count = 0
    size = 128
    ini_time = datetime.now()

    gc.set_range("width", 9, 9)
    gc.set_range("offset", 0, 0)
    #gc.set_range("ext_offset", 6390, 6410)
    #gc.set_range("ext_offset", 6404, 6406)
    #gc.set_range("ext_offset", 6404, 6404)
    gc.set_range("ext_offset", 6404, 6404)
    gc.set_range("tries", 0, num_tries)
    gc.set_range("address", 0x47200 if not in_list else 0, 1024*1024 if not in_list else len(address_list) - 1)
    gc.set_global_step(1)
    gc.set_step("tries", 1)
    gc.set_step("width", 1)
    gc.set_step("ext_offset", 1)
    gc.set_step("address", size if not in_list else 1)

    logger.info("Start glitching")
    for glitch_setting in gc.glitch_values():
        scope.glitch.width = 45
        scope.glitch.repeat = glitch_setting[0]
        scope.glitch.offset = glitch_setting[1]
        scope.glitch.ext_offset = glitch_setting[2]

        if glitch_setting[4] == 0:
            total_successes += successes
            successes = 0
            resets = 0
            #Device is slow to boot?
            reboot_flush(scope, target)
            #if total_successes > max_successes:
            #    break
        target.flush()
        if scope.adc.state:
            # can detect crash here (fast) before timing out (slow)
            logger.debug("Trigger still high!")
            #gc.add("reset")

            #Device is slow to boot?
            reboot_flush(scope, target)
            resets += 1

        scope.arm()
        
        address =  glitch_setting[3] if not in_list else address_list[glitch_setting[3]]
        act(scope, target, address, size)

        ret = scope.capture()

        scope.io.vglitch_reset()
        if ret:
            logger.debug('Timeout - no trigger')
            #gc.add("reset")
            resets += 1

            #Device is slow to boot?
            reboot_flush(scope, target)
        else:
            val = target.simpleserial_read_witherrors('r', 4, glitch_timeout=10, timeout=50)
            valid = val['valid']
            if not valid:
                #logger.debug("reset")
                reboot_flush(scope, target)
                resets += 1
            else:
                data = unpack("<I", val['payload'])[0]
                if data == 0x3400: #for loop check
                    logger.debug("success")
                    logger.debug((scope.glitch.width, scope.glitch.offset, scope.glitch.ext_offset, address))
                    successes += 1
                else:
                    #logger.debug("unsuccessfull")
                    gc.add("normal")
            if with_plt:
                trace = scope.get_last_trace()
                plt.plot(trace, alpha=0.1, color='black')
        count += 1
        if check and count == num_tries:
            break
        #if successes > 0:
        #    logger.info(f"successes = {successes}, resets = {resets}, offset = {scope.glitch.offset}, width = {scope.glitch.width}, repeat = {scope.glitch.repeat}, ext_offset = {scope.glitch.ext_offset}")
        #    total_successes += successes
        if resets > 0:
            logger.debug(f"Reset: offset = {scope.glitch.offset}, width = {scope.glitch.width}, repeat = {scope.glitch.repeat}, ext_offset = {scope.glitch.ext_offset}, address={address:08x}")
        if ini_time + timedelta(seconds=10) <= datetime.now():
            ini_time = datetime.now()
            logger.debug(f"Situation: offset = {scope.glitch.offset}, width = {scope.glitch.width}, repeat = {scope.glitch.repeat}, ext_offset = {scope.glitch.ext_offset}, address={address:08x}")
    logger.info("Done glitching")

    results = gc.calc(ignore_params=["tries", "ext_offset"], sort="success_rate")
    logger.info(results)
    logger.info(f"Adc rate: {scope.clock.adc_rate}")
    if with_plt:
        plt.show()


def app(args):
    scope = cw.scope()
    if not check:
        scope.vglitch_setup('both')
    else:
        scope.vglitch_setup('disabled')
    #scope.glitch.output = "glitch_only"

    if not check:
        scope.clock.clkgen_freq = 200E6
        scope.clock.adc_mul = 0
    else:
        scope.adc.samples = 20000
        scope.clock.clkgen_freq = 100E6
        scope.clock.adc_src = 'clkgen_x1'
    scope.glitch.output = "enable_only"
    if not scope.try_wait_clkgen_locked(10):
        raise OSError("Could not lock PLL. Try rerunning this function or calling scope.pll.reset(): {}".format(scope))

    target = cw.target(scope, cw.targets.SimpleSerial2)

    #scope.io.tio1 = "serial_rx"
    #scope.io.tio2 = "serial_tx"
    #scope.io.tio3 = "high_z"
    #scope.io.tio4 = "high_z"
    #scope.io.glitch_hp = False
    #scope.io.glitch_lp = True
    #scope.glitch.clk_src = "clkgen"
    #scope.glitch.output = "glitch_only"
    #scope.glitch.trigger_src = "ext_continuous"
    #scope.trigger.triggers = "tio4"
    scope.adc.timeout = 0.2
    cw.set_all_log_levels(cw.logging.CRITICAL)
    try:
        glitch_main(scope, target)
    finally:
        cw.set_all_log_levels(cw.logging.WARNING)
        scope.io.tio1 = "high_z"
        scope.io.tio2 = "high_z"
        scope.io.tio3 = "high_z"
        scope.io.tio4 = "high_z"
        scope.glitch_disable()
        scope.dis()
        target.dis()


def _main(argv=sys.argv):
    def auto_int(x):
        return int(x, 0)
    root = logging.getLogger()
    while len(root.handlers) > 0:
        root.removeHandler(root.handlers[0])
    while len(root.filters) > 0:
        root.removeFilter(root.filters[0])
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG,
                        format='%(asctime)s - %(threadName)s - %(name)s - %(levelname)s - %(message)s')
    parser = argparse.ArgumentParser(prog=argv[0], description='CC1352-Glitcher',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-v', '--verbose', dest='verbose_count', action='count', default=0,
                        help="increases log verbosity for each occurrence.")

    # Parse
    args, unknown_args = parser.parse_known_args(argv[1:])

    # Set logging level
    root.setLevel(max(3 - args.verbose_count, 0) * 10)

    return app(args)


# ------------------------------------------------------------------------------

def main():
    try:
        sys.exit(_main(sys.argv))
    except Exception as e:
        logger.exception(e)
        sys.exit(-1)
    finally:
        logging.shutdown()


if __name__ == "__main__":
    main()
