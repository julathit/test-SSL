#! /usr/bin/env python3

from manul.ManulDrive import manul

if __name__=='__main__':
    key_control = manul(1)
    key_control.debug = True
    while True:
        key_control.update()