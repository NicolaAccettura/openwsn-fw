import os
import sys
import struct
import time
here = sys.path[0]
sys.path.insert(0,os.path.join(here,'..','..','..','coap'))

from coap import coap

MOTE_IP = 'bbbb::12:4b00:03a5:'

def csensors_get(mote_ip,coap_object,coap_resource):
    p = coap_object.GET('coap://[{0}]:5683/s/{1}'.format(mote_ip,coap_resource))
    p = struct.pack('>BB',p[0],p[1])
    return p

def csensors_cputemp(mote_ip,coap_object):
    return_value = csensors_get(mote_ip,coap_object,'c')
    return_value = struct.unpack('>H',return_value)[0]
    return_value = ((return_value * 0.58134 - (827 - (25 * 0.58134  * 4.2))) / (0.58134  * 4.2))
    return return_value
    
def csensors_temperature(mote_ip,coap_object):
    return_value = csensors_get(mote_ip,coap_object,'t')
    return_value = struct.unpack('>H',return_value)[0]
    return_value = -46.85 + 175.72 * return_value / 65536
    return return_value
    
def csensors_humidity(mote_ip,coap_object):
    return_value = csensors_get(mote_ip,coap_object,'h')
    return_value = struct.unpack('>H',return_value)[0]
    return_value = -6.0 + 125.0 * return_value / 65536
    return return_value
    
def csensors_light(mote_ip,coap_object):
    return_value = csensors_get(mote_ip,coap_object,'l')
    exponent = struct.unpack('>B',return_value[0])[0]
    if exponent == 0x0f:
        exponent = 0x0e
    mantissa = struct.unpack('>B',return_value[1])[0]
    return_value = 0.045 * 2**exponent * mantissa;
    return return_value
    
def main():
    c = coap.coap()
    control_value = 'x'
    
    while control_value != 'q':
        
        mote_ip_trailer = raw_input('Enter last 2 bytes of IPv6 address > ')
        if not mote_ip_trailer:
            mote_ip = MOTE_IP + '90e7'
        print 'Press a value among the following:'
        print 'c --> cpu temperature sensor'
        print 't --> temperature sensor'
        print 'h --> humidity sensor'
        print 'l --> light sensor'
        print 'g --> go back and insert another IPv6 address'
        print 'q --> quit this app'
        
        while control_value != 'q':
            
            control_value = raw_input('> ')
            if control_value == 'c':
                return_val = csensors_cputemp(mote_ip,c)
                print '\n{0} C'.format(return_val)
            elif control_value == 't':
                return_val = csensors_temperature(mote_ip,c)
                print '\n{0} C'.format(return_val)
                time.sleep(1)
                print '\noooops, I should say...'
                time.sleep(1)
                print '...'
                time.sleep(1)
                print 'calculating...'
                time.sleep(1)
                print '...'
                time.sleep(1)
                print 'calculating...'
                time.sleep(1)
                print '...'
                time.sleep(1)
                print 'calculating...'
                time.sleep(1)
                print '...'
                time.sleep(1)
                print '\n{0} F'.format(return_val*1.8+32)
            elif control_value == 'h':
                return_val = csensors_humidity(mote_ip,c)
                print '\n{0} %'.format(return_val)
            elif control_value == 'l':
                return_val = csensors_light(mote_ip,c)
                print '\n{0} lm'.format(return_val)
            elif control_value == 'q':
                continue
            elif control_value == 'g':
                break
            else:
                print 'press a correct value'
        
    c.close()
    
    #raw_input("Done. Press enter to close.")

#============================ main ============================================

if __name__=="__main__":
    main()