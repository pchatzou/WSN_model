exception_code = -255

import math

import itertools

import re

import ast

import struct

import random

import wx

import wx.xrc

class tunables:

	"""all global values that can be tuned through GUI"""

	all = {'dt':0.001,\
		  'avg_temperature':10,\
		  'avg_humidity':70,\
		  're_link_rate':30,\
		  'max_stack_pointer':100,\
		  'server_forgetting_history_exponent':-2,\
		  'Boltzmann const':1.381e-23,\
		  'carrier wave length':0.8,\
		  'active bandwidth':10,\
		  'additional noise':0\
		  }
		  

class environment:

	"""generalization of environment cases. All units are in SI"""
	
	env_IDs = 0
	
	def __init__(self):
	
		"""every environment object ID is equal with its owing node's ID.
		this is the superclass-general case- for all environment type choices"""
	
		self.properties = {'acceleration' : [1,1,1], 'temperature' : 1, 'humidity' : 1, 'orientation' : [1,1,1]}
		self.k_factor = 1
		self.analogy_factor = 1
		self.velocity = [1,1,1]
		self.position = [1,1,1]
		self.interference = 0
		self.iterations = 0
		environment.env_IDs += 1
		self.ID = environment.env_IDs
		
	def next_values_of(self):
	
		"""environment values being affected for all environment types for one iteration
		dt defined in class tunables is the physical iteration duration"""
		
		#tuning
		dt = tunables.all['dt']
		avg_temp = tunables.all['avg_temperature']
		avg_humidity = tunables.all['avg_humidity']
		#end of tuning
		
		self.properties['acceleration'] = [random.uniform(-5,5) for k in range(3)]
		self.properties['orientation'] = [random.uniform(-180,180) for k in range(2)]
		self.properties['temperature'] = random.gauss(avg_temp, 2)
		self.properties['humidity'] = random.gauss(avg_humidity,0.7)
		self.analogy_factor = 1
		self.k_factor = random.gauss(7,0.1)
		self.environment_specific()
		
	def environment_specific(self):
		pass
		
		
		
class random_environment(environment):

	def __init__(self):
		
		"""case of fully random environmental values"""
		
		environment.__init__(self)
	
	def environment_specific(self):
		"""average values and bounds might need tuning
		dt is cycle duration in secs, everything in the interime is thought 
		as a normal move with std velocity and accl"""
		#tuning
		dt = tunables.all['dt']
		#end of tuning
		self.position = [0.5*k[0]*dt**2 + k[1]*dt +k[2] for k in list(zip(self.properties['acceleration'],self.velocity,self.position))]
		self.velocity = [k[0]*dt + k[1] for k in list(zip(self.properties['acceleration'],self.velocity))]
		
		#self.write_to_txt()
			
	def write_to_txt(self):
	
		with open('environment{0}.txt'.format(self.ID), mode = 'a', encoding = 'utf-8') as log:
			cursor = ''
			for i in self.properties:
				cursor += '\t'*4
				value = self.properties[i]
				if type(value) == int or type(value) == float:
					to_string = '\n{0}{1:.2f}'.format(cursor,value)
					log.write(to_string)
				else:
					for j in value:
						to_string = '\n{0}{1:.2f}'.format(cursor,j)
						log.write(to_string)
						
class clustered_environment(environment):

	def __init__(self, position, mobility = [0,0,0]):
	
		"""building randomly moving clusters around a central point
		For the same central point, node belongs in the same cluster.
		Else is a new cluster or node. Mobility ranges over 3 dimensions
		are the limits of the stochastic displacement next to central position.
		Thus can build either determ or stochastic clusters."""
		
		environment.__init__(self)
		self.central_position = position
		self.position = position
		self.mobility = mobility
		
	def environment_specific(self):
	
		"""update values that apply only to this environment type"""
	
		self.position = [random.uniform(x[0]-x[1],x[0]+x[1])\
						for x in list(zip(self.central_position, self.mobility))]
		
						
class node:

	"""general node case, superclass for state machine"""
	
	active_nodes = {}
	
	def __init__(self, an_environment):
	
		self.environment = an_environment
		self.ID = -1	
		self.link_ID = -1
		self.stack = []
		self.stack_tax = 0
		self.route_cost = 0
		self.hardware = hardware_elems()
		self.active_nodes[self.ID] = self
	
	def time_step(self):
	
		pass
		
class uninitialized(node):

	"""initial state (state one of two) of node state machine"""
		
	def __init__(self, an_environment):
	
		node.__init__(self, an_environment)
		self.ID = -1
		self.stack_tax = 100000
		self.stack.append('init_node=0')
		self.stack.append('broadcast=-1=0')
		
	@classmethod
	def time_step(self, a_node):
	
		"""every 15 steps re transmitting ID request
		if not initialized yet. Until then, does not serve 
		network normally. Neither as a hopping station, nor as sensor box"""
	
		receiving = protocol(a_node, False)
		transmitting = protocol(a_node,True)
		if not a_node.hardware.is_healthy(): 
			del node.active_nodes[a_node.ID]
		if a_node.hardware.cycles_counter % 15 == 0: 
			a_node.stack.append('init_node=0')
			a_node.stack.append('broadcast=-1=0')
		node.active_nodes[a_node.ID]  = a_node
			
			
class initialized(node):

	"""final state (state two of two) of node state machine"""

	def __init__(self, ID, an_environment):
	
		node.__init__(self, an_environment)
		self.ID = ID
		self.active_nodes[self.ID] = self
		
	@classmethod
	def time_step(self, a_node):
	
		'''acts done in every time iteration when initialized'''
		
		a_node.environment.next_values_of()
		a_node.node_specific_functions()
		receiving = protocol(a_node, False)
		transmitting = protocol(a_node,True)
		node.active_nodes[a_node.ID]  = a_node
		
	def node_specific_functions(self):
		pass
		
		

class node_mgmt(uninitialized,initialized):

	"""Physical case of a sensor box. Inherits from one of the two states according to its state."""

	def __init__(self, an_environment):
	
		uninitialized.__init__(self, an_environment)
		self.sampling_rates = [30,30,30,30]
		accelerometer = sensor_is('acceleration', self.environment)
		gyroscope = sensor_is('orientation', self.environment)
		thermometer = sensor_is('temperature', self.environment)
		humiditometer = sensor_is('humidity', self.environment)
		self.sensors = [accelerometer, gyroscope, thermometer, humiditometer]
		
	def time_step(self):
		
		if self.ID == -1:
			uninitialized.time_step(self)
		else:
			initialized.time_step(self)
		
	def node_specific_functions(self):
	
		"""when initialized: 
		1.checking hardware integrity
		2.collecting measurements, if it is time to do so - master clock is system's iterations"""
	
		#tuning
		re_link_rate = tunables.all['re_link_rate']
		#end of tuning
		
		self.handle_stack_cost()
		if self.hardware.is_healthy():
			dying = self.hardware.battery()
		else:
			self.active_nodes.pop(self.ID)
			return 
		if dying:self.stack_tax = 100000
		if self.hardware.cycles_counter % re_link_rate == 0: 
			self.stack.insert(0,'broadcast={}=0'.format(self.ID))
		i=0
		for k in self.sensors:
			if k.get_measurement(self.environment):
				self.stack.insert(0,k.value)
			k.sampling_rate = self.sampling_rates[i]
			i += 1
		
			
	def handle_stack_cost(self) :
	
		'''define how much will be added in route cost other nodes see,
		if node is used as a hopping station according to battery level and how full stack is'''
		#tuning_curves
		full = tunables.all['max_stack_pointer'] 
		knee0, knee1, knee2, knee3 = 0, full / 4, 3 * full / 4, full - 1
		tax0, tax1, tax2, tax3=1, 1, 7, 10000
		#end of tuning
		
		
		knees = [[knee0,tax0],[knee1,tax1],[knee2,tax2],[knee3,tax3]]
		full_percentage = len(self.stack)/ full
		self.stack_tax = int(linear(knees, full_percentage))
	
				
		
class server(initialized):

	"""knees is changing rates for sensors as they are given 
	in node_mgmt (accl, ornt, temp, hum). Need Nyquist 
	Forgetting history. works as prev +histry^forgetting
	Server's position is allways at [0,0,0] and radius of mobility 
	sphere also 0, so it is allways the start of axes"""


	def __init__(self):
		
		initialized.__init__(self, 0, clustered_environment([0,0,0], [0,0,0]))#server is always at [0,0,0]
		self.active_IDs = 0
		self.memory = {}
		
	def time_step(self):
	
		"""an overload of basic synchronous function"""
	
		initialized.time_step(self)
		
	def node_specific_functions(self):
	
		for sensor_box_IDs, sensor_box in self.memory.items():
			for in_sensor in [i for i, x in enumerate(sensor_box.measurements)]:
				it_changes, ID, new_rate = sensor_box.set_rate(in_sensor)
				if it_changes: self.change_rate(ID, in_sensor, new_rate)
		self.hardware.cycles_counter += 1
				
			#write_to_txt(sensor_box)
		
	def prepare_memory(self, rec_route):
		
		"""Physical case of a server object. Only one server supposed to be present"""
		
		self.active_IDs += 1
		self.memory[self.active_IDs] = server_memory_slot(self.active_IDs, rec_route)
				
					
	
	def change_rate(self, ID, sensor, new_rate):
	
		"""prepare a message and push it to network stack to set rate
		at node->ID, sensor->sensor """
										
		cmd = [False,False,False,False,True]
		cmd[1:3] = tobits(sensor)[-3:-1]
		message = cmd + tobits(0) + tobits(ID) + tobits(float(new_rate),float)
		self.stack.insert(0,message)
		
	def write_to_txt(self):
		
		"""write old measurements in txt file in proper indentation
		style is:
		horz:sensor1, sensor2, ...
		vert:dim1
			dim2"""
			
		for ID in range(1,self.active_IDs+1):
			cursor = '' 
			with open('measurements{0}.txt'.format(ID), mode = 'a', encoding = 'utf-8') as log:
				for sensor in self.measurements[ID]:
					for dim in sensor:
						cursor += '\t'*4
						if len(dim) == 3:
							value = dim.pop(-3)
							to_string = '\n{0}{1:.4f}'.format(cursor,value)
							log.write(to_string)
							
class server_memory_slot:

	"""Projection of a sensor box physical entity in server memory.
	Server memory regarding measurements is thus a set of this kind of objects.
	Class property knees refers to linear interpolation of curves regarding the setting of sampling rates"""

	#sampling change rates for all sensors-
	# selective behaviour according to sensors where is :
	#0- temp, 1- hum, 2 - accl, 3- gyroscope:
	#tuning_curves:
	knees = [[[0,20],[8,20],[8.0001,10],[15,10]], [[0,20],[8,20],[8.0001,10],[15,10]],\
			[[0,40],[8,40],[8.0001,20],[15,20]], [[0,40],[8,40],[8.0001,20],[15,20]]]

	#end of tuning


	def __init__(self, ID, route_back):
	
		""" memory management to avoid multi-level list nesting.
		Each object is the image of a sensor box in server memory
		Ideally-future release in case of many types of sensor boxes
		info about sensor_box should be sent with init_node request"""
		
		self.ID = ID
		sensor_box = node.active_nodes[-1]
		self.route_back = route_back
		self.measurements = [[exception_code for x in range(count.dim)] for count in sensor_box.sensors]
		self.previous = self.measurements
		self.changing_history = [[1 for x in range(count.dim)] for count in sensor_box.sensors]
		
	def set_rate(self, in_sensor):
	
		"""keep a record of sensor msmnts changing rates
		accordingly set sensor sampling rate. """
		
		forgetting_history = tunables.all['server_forgetting_history_exponent']
		it_changes = False
		min_new_rate = 10000
		
		for dim in [i for i,x in enumerate(self.measurements[in_sensor])]:
			if self.measurements[in_sensor][dim] == exception_code or self.previous[in_sensor][dim] == exception_code:
				return it_changes, self.ID, min_new_rate #this sensor has not taken initial measurements yet
				
			prev_rate = linear(self.knees[in_sensor], self.changing_history[in_sensor][dim])
			changing_history = self.changing_history[in_sensor][dim]** forgetting_history \
							 + self.measurements[in_sensor][dim] - self.previous[in_sensor][dim]
													
			self.changing_history[in_sensor][dim] = abs(changing_history)
			if self.changing_history[in_sensor][dim]< 1: self.changing_history[in_sensor][dim] = 1
						
			new_rate = linear(self.knees[in_sensor], self.changing_history[in_sensor][dim])
			if new_rate != prev_rate: it_changes = True
			if new_rate < min_new_rate:
				min_new_rate = new_rate
				
		return it_changes, self.ID, min_new_rate
		
		
		

class link:

	"""physical link simulation. transmission equation simulates distance losses in signal power
	a model of AWGN is used, respecting temperature and an extra amount of noise added from electronics
	AWGN and interference ar then simply added forming a probability of symbol corruption distribution
	Only two available symbols (representing bits)-> BPSK"""

	active_links = []
	
	def __init__(self, node, something, alternative =exception_code ):
	
		"""initialize a connection object simulating phusical procedures during transmission
		self.degrade is expressed in absolute DBs (not negative). """
		
		self.connected = False
		self.ID1 = node.ID
		if alternative != exception_code:
			self.ID2 = alternative
		else:
			self.ID2 = node.link_ID
		if self.ID1 not in list(node_mgmt.active_nodes.keys()): return 
		if self.ID2 not in list(node_mgmt.active_nodes.keys()): return 
		
		self.degrade = 1
		self.bulk = something
		self.corrupt(node)
		self.active_links.append(self)
		self.connected = True
		
	def corrupt(self, node):
	
	
		"""corrupt signal with current model according to SIR and SNR
		working with numbers, converting to dB in the end. self.degrade 
		is in absolute dB and is used for external purposes"""
		#tuning
		Boltzmann = tunables.all['Boltzmann const']
		wave_length0 = tunables.all['carrier wave length']
		BW = tunables.all['active bandwidth']
		further_noise = tunables.all['additional noise']
		#end of tuning
		
		white_noise = 10 * math.log10(BW * node.environment.properties['temperature'] * Boltzmann )+\
					  further_noise
		b_node = node.active_nodes[self.ID2]
				  
		dist = node.environment.analogy_factor * \
			   distance(node.environment.position,\
			   b_node.environment.position)
		if dist == 0:
			self.degrade = 0 
		else:
			degrade = 10 * node.environment.k_factor * math.log10(wave_length0 / dist)
			self.degrade = abs(degrade)
			
		receiving_at = node.hardware.transmitting_power - self.degrade
		SNR = receiving_at - white_noise
		SIR = node.environment.interference
		self.noise_attack(SNR,SIR)
		
		
	
	def noise_attack(self, SNR, SIR):
		
		"""signal corruption in channel. BER is given in dB.
		model curves used to map noise and interference level to signal corruption probability
		are subject of practical experimenting, only qualitative and might not be realistic."""
		#tuning_curves
		x0, y0 = 0, -0.7
		
		if SIR == 0:
			x1, y1, x2, y2 = 2, -2, 4, -10
			knees = [[x0,y0],[x1,y1],[x2,y2]]
		else:
			y1 = (10 - SIR / 10) /10
			x1, y1 = 2, -y1
			knees = [[x0, y0],[x1, y1]]
		#end of tuning	
		
		BER = linear(knees, SNR)
		BER = 10 ** (BER / 10)
		self.bulk = [not i if poisson_test(BER) else i for i in self.bulk]		
		
	

		
		
class sensor_is:

	"""Physical sensor model in sensor box. name must be same as in environment dictionary of physical 
	entities (e.g. acceleration, velocity, etc), see environment class->
	instance environment.properties. Sampling rate is given in clock cycles"""

	def __init__(self, name, env):
		
		self.name = name
		self.sampling_rate = 30
		self.cycles_counter = 0
		self.error = 0
		self.value = ''
		if type(env.properties[self.name]) == list:
			self.dim = len(env.properties[self.name])
		else:
			self.dim = 1
	
	def get_measurement(self, env):
	
		""" update self to get measurement as it is in environment object, if it is time to sample"""
		
		if self.cycles_counter % self.sampling_rate == 0 :
			self.value = self.name + '={}'.format(env.properties[self.name]) #auta mpainoune se buffer. Apo kei ta p h s_box
			self.cycles_counter +=1
			return True
		else:
			self.cycles_counter += 1
			return False				
			
			

		
		
class hardware_elems:

	
	def __init__(self):

		"""transm p se dB"""
		
		self.battery_curent = 100
		self.healthy = True
		self.battery_wareout = 0 
		self.cycles_counter = 0
		self.transmitting_power = -5
		
		
		
	def is_healthy(self, die = False):
	
		'''calculate health with regard to duty cycles
		... according to modified bathtub curve
		... possibilities are expressed in absolutely - not %'''
		
		#tuning_curves:
		knee0, knee1, knee2, knee3 = 0, 10 ** 8, 10 ** 13, 2 * 10 ** 13
		
		P0, P1, P2, P3, P4 = 0.005, 0.0005, 0.001, 0.01, 0.10
		
		knees = [[knee0, P0], [knee1, P1], [knee2, P2], [knee3, P3]]
		#end of tuning
		
		failure_probability = linear(knees, self.cycles_counter)
		self.healthy = not poisson_test(failure_probability)
		self.cycles_counter += 1
		if not self.healthy or die :
			return False
		else :
			return True
			
			
	def battery(self ):
	
		'''affect battery_curent current according to duty cycles
		... knee in battery_curent is 10^13 cycles. 
		...Another 1000 cycles after knee till I = 0'''
		
		#tuning_curves:
		knee0, knee1, knee2 = 0, 10 ** 13, 5 * 10 ** 13
		
		knees = [[knee0, 100], [knee1, 100], [knee2, 0]]
		#end of tuning
		self.battery_curent = linear(knees, self.battery_wareout)
		self.battery_wareout += 1
		if self.battery_curent <= 10 :
			return True
			if self.battery_curent <=1:
				self.health(True )
		else:
			return False
		
		
### -------------

###--------------

### protocol utilities
						
class network_policies:

	#__metaclass__ = ABCMeta
		
	@classmethod
	def specify(cls, a_node, packet):
		try:
			command = tuple(packet[:protocol.command_length])
			action = protocol.commands[command](a_node, packet)
		except KeyError:
			action = network_policies.corrupt_packet(a_node)
			
		return action
	
	@classmethod
	def decompose(cls, packet):
	
		"""/decompose received packet in a list of all its parts according to protocol"""
	
		br = [0, \
			  protocol.command_length, \
			  protocol.command_length + protocol.int_length, \
			  protocol.command_length + 2 * protocol.int_length]
		next_br = itertools.cycle(br)
		next(next_br)
		contents = [packet[k[0]:k[1]] for k in zip(br, next_br)]
		contents.pop(-1)
		
		contents.append(packet[br[-1]:br[-1]+protocol.message_length])
		
		route = [packet[x:x+protocol.int_length] for x in \
				 range(br[-1]+protocol.message_length, len(packet), protocol.int_length)]
		contents.extend(route)
		
		return contents
	
	@classmethod
	def cmd(cls, command):
	
		"""analyze command in its parts (see WSNprotocol0)"""
		
		br = [protocol.indicative,\
			  protocol.indicative+protocol.enumerative,\
			  protocol.indicative+protocol.enumerative+protocol.per_se \
			  ]
		ans = [command[:br[0]],command[br[0]:br[1]], command[br[1]:br[2]]]
		return ans
	
	@classmethod
	def backwards(cls, contents, a_node):
	
		"""forwards any message to the source that sent it"""
		
		a_node.hardware.battery_wareout +=  10 ** (a_node.hardware.transmitting_power / 10) * 10
		 
		contents[1] = contents.pop(-1) 
		packet = list(itertools.chain(*contents))
		connect = link(a_node, packet, tovalue(contents[1]))
	
	@classmethod
	def forwards(cls, contents, a_node):
	
		"""forwards any message to server via next link ID"""
		a_node.hardware.battery_wareout +=  10 ** (a_node.hardware.transmitting_power / 10) * 10
		
		contents[1] = tobits(a_node.link_ID)
		packet = list(itertools.chain(*contents))
		packet.extend(tobits(a_node.ID))
		connect = link(a_node, packet)
	
	@classmethod
	def ping_back(cls, srvr, source):
		
		"""doesn't work for lists command at once. might work at more rounds
		must take command under process as input to find route back"""
		
		route_back = srvr.memory[source].route_back
		if type(srvr.stack[-1]) == str: 
			protocol.prepare(srvr)
		packet = srvr.stack.pop(-1)
		for x in route_back: packet += x
		fwd_contents = network_policies.decompose(packet) 
		network_policies.backwards(fwd_contents, srvr)
		
	@classmethod
	def corrupt_packet(cls, a_node):
		
		"""insert to stack packet statement about error to be displayed in history log"""
		
		a_node.stack.insert(0,'corrupt packet')

		
		
class init_node(network_policies):

	def __init__(self, a_node, packet):
			
		"""sets with forwarded ID the first a_node it meets"""
		
		contents = self.decompose(packet)
		if a_node.ID == -1 and tovalue(contents[3]) != 0:
			a_node.ID = int(tovalue(contents[3]))
			this = node_mgmt.active_nodes.pop(-1)
			node_mgmt.active_nodes[a_node.ID] = this
		elif a_node.ID!=0 and tovalue(contents[3]) == 0:
			self.forwards(contents, a_node)
		elif a_node.ID == 0 and tovalue(contents[3]) == 0:
			a_node.prepare_memory(contents[4:])
			a_node.stack.append('init_node=-1={0}'.format(a_node.active_IDs))
			self.ping_back(a_node, a_node.active_IDs)
		else:
			self.backwards(contents, a_node)
		
			
class sensor(network_policies):

	def __init__(self, a_node, packet):
	
		"""forwards sensor data to server"""

		contents = self.decompose(packet)

		if a_node.ID == 0 :
			command = self.cmd(contents[0])
			source_ID = tovalue(contents[4])
			specific_sensor = tovalue(command[-1]+[False]) #note when refactoring tobits
			dimension = tovalue(command[-2]+[False])
			if a_node.memory[source_ID].measurements[specific_sensor][dimension] != exception_code: #if it has got initial values
				a_node.memory[source_ID].previous[specific_sensor][dimension]= a_node.memory[source_ID].measurements[specific_sensor][dimension]
			a_node.memory[source_ID].measurements[specific_sensor][dimension] = tovalue(contents[3])
			a_node.memory[source_ID].route_back = contents[4:]
		else:
			self.forwards(contents, a_node)

	
	
class set_rate(network_policies):

	def __init__(self, a_node, packet):
	
		"""case where sampling rates need to be set"""
		
		contents = self.decompose(packet)
		command = self.cmd(contents[0])

		if tovalue(contents[2]) == a_node.ID and a_node.ID != 0 :
			a_node.sampling_rates[tovalue(command[1])] = int(tovalue(contents[3]))
		elif a_node.ID == 0:
			self.ping_back(a_node, tovalue(contents[2]))
			#a_node.route_back[tovalue(contents[4]) = contents[4:]
			#server stuff, should be utilized in server class. will sure include a ping back
		else:
			self.backwards(contents, a_node)
			
			
class broadcast(network_policies):

	def __init__(self, a_node, packet):
	
		"""broadcast works autonomsly. Unlike other network_policies,
		it creates the links to all currently active nodes ITSELF.
		If a_node is reached by a broadcast signal, responds immidiately
		the orthodox way. So it is represented that all links are in 1 cycle"""
	
		contents = self.decompose(packet)

		if tovalue(contents[2]) == a_node.ID and tovalue(contents[3]) == 0:
			self.link_to_server(a_node, contents)
		elif tovalue(contents[2]) == a_node.ID and tovalue(contents[3]) == 100: #in fact is the a_node[ID].route_cost, !=0
			pass # supposed to ping back info about each link here, gets info in prev case instead
		else:
			contents[3] = tobits(100,float)
			self.backwards(contents, a_node)

			
	def link_to_server(self, a_node, contents):
	
		"""if broadcast message is 0 means it sends to request a new link"""
		#tuning_curves
		#first is decay in dB, second the tax
		degrade_taxes = [[0,1], [20,1], [40,10], [80,30], [120, 50]]
		#end of tuning
		
		a_node.hardware.battery_wareout +=  10 ** (a_node.hardware.transmitting_power / 10) * 10
		costs = []
		for ID in list(node.active_nodes.keys()):
			if ID != a_node.ID :
				contents[1] = tobits(ID)
				packet = list(itertools.chain(*contents))
				connect = link(a_node, packet, ID)
				if connect.connected:
					cost = linear(degrade_taxes, connect.degrade) + \
					   node.active_nodes[ID].route_cost + \
					   node.active_nodes[ID].stack_tax
					costs.append([round(cost),ID])			
					d_values = [d[0] for d in costs]
					d_IDs = [d[1] for d in costs]
					a_node.link_ID = d_IDs[d_values.index(min(d_values))]
					a_node.route_cost = min(d_values)

					
class protocol:

	""" if sending == false node is receiving. Defining protocol fields length.
	Protocol message is cmd;next_node;final_destination;message:route_trace1:route_trace2
	..for further info for protocol see WSN protocol0.xlsx """
				
	command_length = 5
	#of which:
	indicative = 1
	enumerative = 2
	per_se = 2
	#
	message_length = 32
	int_length = 9
	
	commands = {(True,False,False,False,False):	sensor,						\
				(True,False,True,False,False):	sensor, 					\
				(True,True,False,False,False):	sensor, 					\
				(True,False,False,False,True):	sensor,	   					\
				(True,False,True,False,True):	sensor,    					\
				(True,True,False,False,True):	sensor,	   					\
				(True,False,False,True,False):	sensor,						\
				(True,False,False,True,True):	sensor, 					\
				(False,False,False,False,False):init_node,					\
				(False,False,False,False,True):	set_rate, 					\
				(False,False,True,False,True):	set_rate, 					\
				(False,True,False,False,True):	set_rate, 					\
				(False,True,True,False,True):	set_rate, 	  				\
				(False,False,False,True,False):	broadcast					\
				}
				
	reverse = {'acceleration':[[True,False,False,False,False],				\
							   [True,False,True,False,False], 				\
							   [True,True,False,False,False] 				\
							   ], 											\
				'orientation':[[True,False,False,False,True],			 	\
							   [True,False,True,False,True], 				\
							   [True,True,False,False,True]  				\
							   ], 											\
				'temperature':[True,False,False,True,False], 				\
				'humidity'	 :[True,False,False,True,True], 				\
				'broadcast':[False,False,False,True,False],					\
				'init_node':[False,False,False,False,False]					\
				}
				

	def __init__(self, node, sending = True ):
	
		
		""""..preparing packet if not in protocol format to be sent.
		works for format name = value.
		Value can be anythng numeric, list, int, float, tuple
		Strongly depends on sensor name property"""
		
		if sending :
			self.Tx(node)
		else :
			self.Rx(node)
			
	def Tx(self, node):
		
		"""every node has a stack that serves all packets to be transmitted.
		According to case call appropriate class"""
	
		if not node.stack: return
		node.hardware.battery_wareout +=  10 ** (node.hardware.transmitting_power / 10) * 10
		if type(node.stack[-1]) == str: 
			is_not_corrupt = self.prepare(node)	
		else:
			is_not_corrupt = True
		if is_not_corrupt: 
			packet = node.stack.pop(-1)
		else:
			return
		action = network_policies.specify(node, packet)
		
		
	def Rx(self, node):
	
		"""Rx does not handle acquired info. They are handled
		even when they reach goal by Tx as soon as they reach top of stack"""
		
		active_link_IDs = [[x.ID1, x.ID2] for x in link.active_links]
		receiving = [i for i, x in enumerate(active_link_IDs) if x[1] == node.ID]
		if not receiving: return
		l = len(receiving)
		for i in range(l):
			this_link = link.active_links.pop(receiving[-1])
			packet = this_link.bulk
			node.stack.insert(0,this_link.bulk)
			active_link_IDs = [[x.ID1, x.ID2] for x in link.active_links]
			receiving = [i for i, x in enumerate(active_link_IDs) if x[1] == node.ID]
	
	@classmethod
	def prepare(self, node):
	
			"""first all commands are given in str format
			it is cmd=target ID = message value"""
			to_send = node.stack.pop(-1)
			to_send = re.split('=',to_send)
			
			if len(to_send) == 2:
				target = tobits(0)
			elif len(to_send) == 3:
				target = tobits(ast.literal_eval(to_send[-2]))
			elif to_send[0] == 'corrupt packet':
				return False #will be simply dropped from the stack, after having been displayed
				
			name = to_send[0]
			values = ast.literal_eval(to_send[-1])
			if type(values) == list or type(values) == tuple:
				count = 0
				for value in values:
					packet = self.reverse[name][count] + tobits(node.link_ID) + target +  tobits(value,float)
					node.stack.append(packet)
					count +=1 
			elif type(values) == int or type(values) == float:
				packet = self.reverse[name] + tobits(node.link_ID) + target +  tobits(values,float)
				node.stack.append(packet)
			else:
				raise ValueError('improper sensor {} output format'.format(name))
				
			return True

#-----------------------------------------------				
				
#-----------------------------------------------

#general functions used globally

			
			
def tovalue(bits):

	"""Big endian, LSB is sign bit, True for negative"""
	
	negative= bits[-1]
	
	if len(bits) == protocol.message_length :
		bitstring = ['1' if k else '0' for k in bits[:-1]]
		num = int('0b' + ''.join(bitstring), 2)
		s = struct.pack('>l',num)
		my_float = struct.unpack('>f', s)
		ans = float(my_float[0])
	elif len(bits) <= protocol.int_length :
		bitstring = ['1' if k else '0' for k in bits[:-1]]
		ans = int('0b' + ''.join(bitstring), 2)
	else:
		raise ValueError('impoper input format')
		
	if negative:
		return -ans
	else:
		return ans
	
	
	
def tobits(num, to = int):

	"""any number to bits. Operates 8 bit integer input, 61 bit long.
	Big endian format.One extra LSB is dedicated sign bit"""
	
	if num<0: 
		negative = '1'
	else:
		negative = '0'
	
	num = abs(num)
	
	if to == float:
		s = struct.pack('>f',num)
		my_int = struct.unpack('>l',s)
		my_string = bin(int(my_int[0])).lstrip('0b')
		my_string = my_string.zfill(31)
	elif to == int:
		my_string = bin(num).lstrip('0b')
		my_string = my_string.zfill(8)
	else:
		raise ValueError('only float or int should be sent or received')
		
	my_string += negative
	return [True if k == '1' else False for k in my_string]
	
	
def distance(x, y):

	'''calculates euclidean 3D distance'''
	
	import math
	x = list(x)
	y = list(y)
	d=0
	for i in range(3):
		d+=(x[i]-y[i])**2
	d = math.sqrt(d)
	
	return d

	
def linear(knees, x):

	"""linear interpolation. knees is a list of list containing angle points"""
	
	from itertools import cycle
	next_knees = cycle(knees)
	next_one = next(next_knees)
	for one in knees:
		next_one = next(next_knees) 
		m = (next_one[1] - one[1])/(next_one[0]- one[0])
		y = m * x - m * one[0] + one[1]
		if x <= next_one[0]:
			return y
		else:
			if next_one[0] != knees[0][0]:
				prev_y = y
			else:
				return prev_y
	
	
	
def poisson_test(p):

	'''Poisson test with two possible outcomes, where p is failure probability'''
	
	import random
	
	return random.random() <= p
	
	
###-------------

###------------

class topWindow ( wx.Frame ):
	
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"tutorial WSN model", pos = wx.DefaultPosition, size = wx.Size( 645,476 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		#self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		top_level = wx.BoxSizer( wx.VERTICAL )
		
		parametrizeables = wx.BoxSizer( wx.HORIZONTAL )
		
		self.logger = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE|wx.TE_READONLY )
		parametrizeables.Add( self.logger, 2, wx.ALL|wx.EXPAND, 5 )
		
		
		top_level.Add( parametrizeables, 10, wx.ALIGN_CENTER|wx.ALIGN_CENTER_HORIZONTAL|wx.EXPAND, 5 )
		
		bSizer9 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.steps_num = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer9.Add( self.steps_num, 0, wx.ALL, 5 )
		
		self.x_steps = wx.Button( self, wx.ID_ANY, u"x steps", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer9.Add( self.x_steps, 0, wx.ALL, 5 )
		
		self.m_button11 = wx.Button( self, wx.ID_ANY, u"add node", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer9.Add( self.m_button11, 0, wx.ALL, 5 )
		
		m_comboBox2Choices = list(tunables.all.keys())
		self.m_comboBox2 = wx.ComboBox( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, m_comboBox2Choices, 0 )
		self.m_comboBox2.SetSelection( 5 )
		bSizer9.Add( self.m_comboBox2, 1, wx.ALL, 5 )
		
		
		top_level.Add( bSizer9, 1, wx.ALIGN_CENTER|wx.ALIGN_CENTER_VERTICAL, 5 )
		
		
		self.SetSizer( top_level )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.steps_num.Bind( wx.EVT_TEXT, self.num_of_steps )
		self.x_steps.Bind( wx.EVT_BUTTON, self.move_x_steps )
		self.m_button11.Bind( wx.EVT_BUTTON, self.add_node )
		self.m_comboBox2.Bind( wx.EVT_COMBOBOX, self.show_dlg )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def num_of_steps( self, event ):
		event.Skip()
	
	def move_x_steps( self, event ):
		event.Skip()
	
	def add_node( self, event ):
		event.Skip()
	
	def show_dlg( self, event ):
		event.Skip()
		
		
class MyDialog1 ( wx.Dialog ):
	
	def __init__( self, parent, global_var):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = global_var, pos = wx.DefaultPosition, size = wx.DefaultSize, style = wx.DEFAULT_DIALOG_STYLE )
		#########################
		self.affect = global_var
		self.affected_value = 1
		#########################
		
		#self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		bSizer1 = wx.BoxSizer( wx.VERTICAL )
		
		self.value_to_change = wx.StaticText( self, wx.ID_ANY, global_var, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.value_to_change.Wrap( -1 )
		bSizer1.Add( self.value_to_change, 0, wx.ALL, 5 )
		
		self.value_to_change_ctrl = wx.TextCtrl( self, wx.ID_ANY, str(tunables.all[global_var]), wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.value_to_change_ctrl, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		bSizer2 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.apply_button = wx.Button( self, wx.ID_ANY, u"apply", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer2.Add( self.apply_button, 0, wx.ALL, 5 )
		
		self.discard_button = wx.Button( self, wx.ID_ANY, u"discard", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer2.Add( self.discard_button, 0, wx.ALL, 5 )
		
		
		bSizer1.Add( bSizer2, 1, wx.EXPAND, 5 )
		
		
		self.SetSizer( bSizer1 )
		self.Layout()
		bSizer1.Fit( self )
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.value_to_change_ctrl.Bind( wx.EVT_TEXT, self.get_this )
		self.apply_button.Bind( wx.EVT_BUTTON, self.apply )
		self.discard_button.Bind( wx.EVT_BUTTON, self.discard )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def get_this( self, event ):
		event.Skip()
	
	def apply( self, event ):
		event.Skip()
	
	def discard( self, event ):
		event.Skip()

class environment_type_choice ( wx.Dialog ):
	
	def __init__( self, parent ):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"choose environment", pos = wx.DefaultPosition, size = wx.Size( 177,142 ), style = wx.DEFAULT_DIALOG_STYLE )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		bSizer1 = wx.BoxSizer( wx.VERTICAL )
		
		m_radioBox1Choices = [ u"clustered", u"fully random" ]
		self.m_radioBox1 = wx.RadioBox( self, wx.ID_ANY, u"spatial clustering type", wx.DefaultPosition, wx.DefaultSize, m_radioBox1Choices, 1, wx.RA_SPECIFY_COLS )
		self.m_radioBox1.SetSelection( 0 )
		bSizer1.Add( self.m_radioBox1, 0, wx.ALL, 5 )
		
		self.choose_env = wx.Button( self, wx.ID_ANY, u"OK", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.choose_env, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		
		self.SetSizer( bSizer1 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.m_radioBox1.Bind( wx.EVT_RADIOBOX, self.environment_choice )
		self.choose_env.Bind( wx.EVT_BUTTON, self.build_node )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def environment_choice( self, event ):
		event.Skip()
	
	def build_node( self, event ):
		event.Skip()

class clustered_env_params ( wx.Dialog ):
	
	def __init__( self, parent ):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"clustering parameters", pos = wx.DefaultPosition, size = wx.Size( 191,343 ), style = wx.DEFAULT_DIALOG_STYLE )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		bSizer1 = wx.BoxSizer( wx.VERTICAL )
		
		self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"central position", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText1.Wrap( -1 )
		bSizer1.Add( self.m_staticText1, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_textCtrl1 = wx.TextCtrl( self, wx.ID_ANY, u"x", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_textCtrl1, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_textCtrl2 = wx.TextCtrl( self, wx.ID_ANY, u"y", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_textCtrl2, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_textCtrl3 = wx.TextCtrl( self, wx.ID_ANY, u"z", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_textCtrl3, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_staticText2 = wx.StaticText( self, wx.ID_ANY, u"mobility range", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText2.Wrap( -1 )
		bSizer1.Add( self.m_staticText2, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_textCtrl4 = wx.TextCtrl( self, wx.ID_ANY, u"x", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_textCtrl4, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_textCtrl5 = wx.TextCtrl( self, wx.ID_ANY, u"y", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_textCtrl5, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_textCtrl6 = wx.TextCtrl( self, wx.ID_ANY, u"z", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_textCtrl6, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		self.m_button1 = wx.Button( self, wx.ID_ANY, u"Apply", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1.Add( self.m_button1, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 5 )
		
		
		self.SetSizer( bSizer1 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.m_textCtrl1.Bind( wx.EVT_TEXT, self.read_position_x )
		self.m_textCtrl2.Bind( wx.EVT_TEXT, self.read_position_y )
		self.m_textCtrl3.Bind( wx.EVT_TEXT, self.read_position_z )
		self.m_textCtrl4.Bind( wx.EVT_TEXT, self.read_mobility_x )
		self.m_textCtrl5.Bind( wx.EVT_TEXT, self.read_mobility_y )
		self.m_textCtrl6.Bind( wx.EVT_TEXT, self.read_mobility_z )
		self.m_button1.Bind( wx.EVT_BUTTON, self.apply_settings )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def read_position_x(self, event) :
		event.Skip()
		
	def read_position_y(self, event):
		event.Skip()
		
	def read_position_z(self, event):
		event.Skip()
		
	def read_mobility_x(self, event):
		event.Skip()
		
	def read_mobility_y(self, event):
		event.Skip()
		
	def read_mobility_z(self, event):
		event.Skip()
	
	def apply_settings( self, event ):
		event.Skip()
		
#--------------------------------------------------------------
#------------------end of computer generated-------------------
		
class my_topWindow(topWindow):

	def __init__(self,parent = None):
	
		topWindow.__init__(self, parent)
		self.steps = 1
		self.ctrl = main_ctrl()

	@classmethod
	def evaluate_input(cls, the_string):
	
		"""quick n dirty check if all values obtained from text fields are 
		not empty and of nummeric type"""
	
		if not the_string: 
			return False, 0
		try:
			value = ast.literal_eval(the_string)
			if type(value) == int or type(value) == float :
				return True, value
		except ValueError:
			return False, 0
		except SyntaxError:
			return False, 0
				
		
	def show_dlg(self, event):
		global_var = event.GetString()
		get_values = my_MyDialog1(self, global_var)
		get_values.Show(True)
		
	def num_of_steps(self, event):
		steps = event.GetString()
		is_OK, steps = self.evaluate_input(steps)
		if not is_OK: return 
		self.steps = steps
		
		
	def move_x_steps(self, event):
		status = self.ctrl.main_loop(self.steps)
		with open('latest_network_state.txt', mode = 'r') as network_state:
			self.logger.SetValue(network_state.read())
		self.Layout()
		
	def add_node(self, event):
		choose_env = my_environment_type_choice(self)
		choose_env.Show(True)
		
		
class my_MyDialog1(MyDialog1):

	"""sets the global values of class tunables 
	affecting experiment paramteters"""

	def get_this(self, event):
		is_OK, value = my_topWindow.evaluate_input(event.GetString())
		if is_OK: self.affected_value = value

	def apply(self, event):
	
		tunables.all[self.affect] = self.affected_value
		self.Close()
		
	def discard(self, event):
	
		self.Destroy()
		

class my_environment_type_choice(environment_type_choice):

	def __init__(self, parent):
		environment_type_choice.__init__(self, parent)
		self.selected_type = 0
		self.parent = parent #fix to refer to self. parent for the popup dialog. Find how to refer properly and remove

	def environment_choice(self, event):
		self.selected_type = event.GetSelection()
		
	def build_node(self, event):
		if self.selected_type == 0: #clustered
			set_params = my_clustered_env_params(self.parent)
			set_params.Show(True)
		elif self.selected_type == 1:
			main_ctrl.present_nodes.append(node_mgmt(random_environment()))
			
		self.Destroy()
			
		

class my_clustered_env_params(clustered_env_params):

	def __init__(self, parent):
		clustered_env_params.__init__(self, parent)
		self.position = [0,0,0]
		self.mobility = [0,0,0]

	def read_position_x(self, event):
		self.set_input(0,0,event)
		
	def read_position_y(self, event):
		self.set_input(0,1,event)
		
	def read_position_z(self, event):
		self.set_input(0,2,event)
	
	def read_mobility_x(self, event):
		self.set_input(1,0,event)
		
	def read_mobility_y(self, event):
		self.set_input(1,1,event)
		
	def read_mobility_z(self, event):
		self.set_input(1,2,event)
		
	def apply_settings(self, event):
	
		described_environment = clustered_environment(self.position, self.mobility)
		main_ctrl.present_nodes.append(node_mgmt(described_environment))
		self.Destroy()
		
	def set_input(self, which_attribute, dimmension, event):
	
		value = event.GetString()
		input_is_OK, value = my_topWindow.evaluate_input(value)
		if not input_is_OK: return 
		if which_attribute == 0:
			self.position[dimmension] = value
		else:
			self.mobility[dimmension] = value
	
		
			
		

class my_view(wx.App):

	def OnInit(self):
	
		frame = my_topWindow(None)
		frame.Show(True)
		self.SetTopWindow(frame)
		return True
		

		
class main_ctrl:

	present_nodes = [server()]

	def main_loop(self, n):
	
		"""applies main control functions (iterations) for n steps"""
		
		for step in range(n):
			for count in self.present_nodes:
				count.time_step()
				
		#self.nodes = node_mgmt.active_nodes
				
		with open('latest_network_state.txt', mode = 'w') as network_state:
			self.write_log(network_state, style = 'network state')
					
		with open('simulation_history.txt', mode = 'a') as history:
			self.write_log(history, style = 'history')
				
	def write_log(self, to_file, style = 'history'):
	
		"""apply to log current network state.
		history style is more detailed,
		network state is much more minimal"""
	
		if style == 'history':
			system_iteration = self.present_nodes[0].hardware.cycles_counter
			note_iterration = 5*'--' + 'SYSTEM ITERATION ' +str(system_iteration) + 5*'--' + 3*'\n'
			to_file.write(note_iterration)
		for count in self.present_nodes[1:]:
			to_file.write('node{0}:\n\n samp.rates:{1}\n'.format(count.ID,count.sampling_rates))
			if style == 'history':
				note_them = ['link ID is:{0}\n'.format(count.link_ID),\
							'route cost:{0}\n stack tax:{1}\n'.format(count.route_cost,count.stack_tax),\
							'current position is:{0}\n'.format(count.environment.position)\
							]
				for s in note_them: to_file.write(s)
			for stack_item in count.stack:
				to_file.write(self.decode(stack_item))
			to_file.write('\n\n')
					
	def decode(self, stack_item):
	
		"""decodes a stack item to make it in a readable string to be displayed in logger"""
			
		my_string = ''
		copy_item = stack_item
		if copy_item == 'corrupt packet': return copy_item
		if type(copy_item) == str: return 'raw msmt\n'
		contents = network_policies.decompose(copy_item)
		my_string += protocol.commands[tuple(contents[0])].__name__+'\t' + 'next:{0}\t'.format(tovalue(contents[1]))+'fin:{0}\t'.format(tovalue(contents[2]))+\
					'value:{0:.2f}'.format(tovalue(contents[3])) +'\n'
					
		return my_string

if __name__ == '__main__':

	app = my_view(False)
	app.MainLoop()			

