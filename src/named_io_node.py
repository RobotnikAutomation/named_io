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
        self._pub = rospy.Publisher('~named_inputs_outputs', named_inputs_outputs, queue_size=1)

        # service
        self._write_digital_output = rospy.ServiceProxy('robotnik_modbus_io/write_digital_output', set_digital_output)

    def get_digital_output_number(self, req):
        '''
            Returns the output and value if the name exists
        '''
        output = 0
        value = False
        if self._o_dict.has_key(req.name):
            output = self._o_dict[req.name][0]['number']
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
            named_io.value = data.digital_inputs[self._i_dict[key][0]['number'] - 1]
            msg.digital_inputs.append(named_io)
        self._pub.publish(msg)


    def set_named_digital_output_callback(self, req):
        '''
            NamedIO service. It gets the number and the output value, then calls
            to a service from robotnik_modbus_io.
        '''    
        response = set_named_digital_outputResponse()
        response.ret = True

        # set the correct information
        out, value = self.get_digital_output_number(req)
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
            request = set_digital_outputRequest()
            request.output = output
            request.value = value
            response = self._write_digital_output(request)
            return response
        except rospy.ServiceException, e:
            rospy.logerr('write_digital_output service call failed')
            return False


    def setup(self):
        # Set up the params
        self._o_dict = rospy.get_param('outputs')
        self._i_dict = rospy.get_param('inputs')

        service = rospy.Service('named_io/set_named_digital_output', set_named_digital_output, self.set_named_digital_output_callback)
        rospy.Subscriber("robotnik_modbus_io/input_output", inputs_outputs, self.inputs_callback)
        

if __name__ == '__main__':
    rospy.init_node('named_io_node')
    mb = NamedIO()
    mb.setup()
    rospy.spin()
