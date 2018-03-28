#!/usr/bin/env python
import rospy
import itertools
from robotnik_msgs.srv import set_digital_output, set_digital_outputRequest, set_named_digital_output, set_named_digital_outputResponse
from robotnik_msgs.msg import inputs_outputs, named_input_output, named_inputs_outputs

class NamedIO():

    def __init__(self):
        # inputs
        self._i_dict = {}

        # outputs
        self._o_dict = {}

        # publisher
        self._pub = rospy.Publisher('named_inputs_outputs', named_inputs_outputs, queue_size=1)

    def set_digital_output_info(self, req):
        '''
            Returns the output and value if the name exists
        '''
        output = 0
        value = False
        if self._o_dict.has_key(req.name):
                output = self._o_dict[req.name]
                value = req.value 
                return output, value
        rospy.logwarn(req.name + " not exists")
        return None, None

    def inputs_callback(self, data):
        '''
            Publish the input data through a topic
        '''
        msg = named_inputs_outputs()
        msg.digital_inputs = []
        for key in self._i_dict:
            named_io = named_input_output()
            named_io.name = key
            named_io.value = data.digital_inputs[self._i_dict[key] - 1]
            msg.digital_inputs.append(named_io)
        self._pub.publish(msg)


    def set_named_digital_output_callback(self, req):
        '''
            
        '''    
        response = set_named_digital_outputResponse()
        response.ret = True

        # set the correct information
        out, value = self.set_digital_output_info(req)
        if out == None:
            response.ret = False
        else:
            # Call the modbus_io service
            srv_response = self.set_digital_output(out, value)
            if not srv_response:
                response.ret = False
        return response

    def set_digital_output(self, output, value):
        '''
           Function that writes the info received into the write_digital_output service
        '''
        rospy.wait_for_service('robotnik_modbus_io/write_digital_output')
        try:
            write_digital_output = rospy.ServiceProxy('robotnik_modbus_io/write_digital_output', set_digital_output)
            request = set_digital_outputRequest()
            request.output = output
            request.value = value
            response = write_digital_output(request)
            return response
        except rospy.ServiceException, e:
            rospy.logerr('write_digital_output service call failed')
            return False


    def setup(self):
        # Set up the output params
        self._o_dict['e_stop'] = rospy.get_param("outputs/e_stop", default=0)[0]['number']
        self._o_dict['manual_release'] = rospy.get_param("outputs/manual_release", default=0)[0]['number']
        self._o_dict['bumper_override'] = rospy.get_param("outputs/bumper_override", default=0)[0]['number']
        self._o_dict['loaded'] = rospy.get_param("outputs/loaded", default=0)[0]['number']
        self._o_dict['elevator_up'] = rospy.get_param("outputs/elevator_up", default=0)[0]['number']
        self._o_dict['elevator_down'] = rospy.get_param("outputs/elevator_down", default=0)[0]['number']
        self._o_dict['turn_left'] = rospy.get_param("outputs/turn_left", default=0)[0]['number']
        self._o_dict['turn_right'] = rospy.get_param("outputs/turn_right", default=0)[0]['number']

        # Set up the input params
        self._i_dict['s3_elevator_down'] = rospy.get_param("inputs/s3_elevator_down", default=0)[0]['number']
        self._i_dict['s4_elevator_up'] = rospy.get_param("inputs/s4_elevator_up", default=0)[0]['number']
        self._i_dict['e_stop_ch1'] = rospy.get_param("inputs/e_stop_ch1", default=0)[0]['number']
        self._i_dict['e_stop_ch2'] = rospy.get_param("inputs/e_stop_ch2", default=0)[0]['number']
        self._i_dict['k8_elevator_down'] = rospy.get_param("inputs/k8_elevator_down", default=0)[0]['number']
        self._i_dict['k9_elevator_up'] = rospy.get_param("inputs/k9_elevator_up", default=0)[0]['number']

        service = rospy.Service('named_io/set_named_digital_output', set_named_digital_output, self.set_named_digital_output_callback)
        rospy.Subscriber("robotnik_modbus_io/input_output", inputs_outputs, self.inputs_callback)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('named_io_node')
    mb = NamedIO()
    mb.setup()
    