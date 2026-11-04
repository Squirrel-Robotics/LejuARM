"""Standalone Pico joystick/discovery → DDS process, without body retargeting."""

import argparse
import logging
import time

from leju_gmr_common.dds_io import DdsPublishers, JOYSTICK_TOPIC
from .pico_receiver import PicoReceiver


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=12345)
    parser.add_argument('--domain-id', type=int, default=0)
    parser.add_argument('--joy-topic', default=JOYSTICK_TOPIC)
    parser.add_argument('--broadcast-name', default='ROBAN')
    parser.add_argument('--no-broadcast', action='store_true')
    return parser


def main():
    args = build_arg_parser().parse_args()
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    publishers = DdsPublishers(domain_id=args.domain_id, heft_topic=None,
                               joy_topic=args.joy_topic, bone_topic=None, head_topic=None)
    receiver = PicoReceiver(host=args.host, port=args.port,
                            enable_ip_broadcast=not args.no_broadcast,
                            robot_name=args.broadcast_name,
                            joy_callback=publishers.publish_joysticks,
                            receive_body=False)
    try:
        receiver.start()
        while receiver.running:
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


if __name__ == '__main__':
    main()
