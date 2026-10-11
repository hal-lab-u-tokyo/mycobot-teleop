"""Release all myCobot servos through the fixed TCP server."""

from pymycobot import MyCobot280Socket
from mycobot_control import release_all_servos


MYCOBOT_IP = "192.168.50.2"
MYCOBOT_PORT = 9000


def main():
    mc = MyCobot280Socket(MYCOBOT_IP, MYCOBOT_PORT)
    release_all_servos(mc)


if __name__ == "__main__":
    main()
