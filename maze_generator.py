from collections import defaultdict
import numpy as np
import argparse
import random
import pprint
import json

def copy_empty_world(root_path):
    f_in = open(root_path+'/worlds/empty.world', 'r')
    f_out = open(root_path+'/worlds/sphere_maze.world', 'w')
    for line in f_in:
        f_out.write(line)
    f_in.close()
    return f_out

def add_sphere_description(f_out, count):
	for i in range(1, count):
		f_out.write('<model name=\'sphere{}\'>\n'.format(i))
		f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<sphere>\n<radius>0.5</radius>\n</sphere>\n')
		f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
		f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<sphere>\n<radius>0.5</radius>\n</sphere>\n</geometry>\n<material>\n<script>\n')
		f_out.write('<name>Gazebo/Grey</name>\n<uri>file://media/materials/scripts/gazebo.material</uri>\n</script>\n<shader type=\'pixel\'><normal_map>__default__</normal_map>\n</shader>\n')
		f_out.write('<ambient>1 0 0 1</ambient>\n<diffuse>0.7 0.7 0.7 1</diffuse>\n<specular>0.01 0.01 0.01 1</specular>\n<emissive>0 0 0 1</emissive>\n</material>\n')
		f_out.write('<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<transparency>0</transparency>\n<cast_shadows>1</cast_shadows>\n</visual>\n<self_collide>0</self_collide>\n')
		f_out.write('<kinematic>0</kinematic>\n<gravity>0</gravity>\n</link>\n<pose frame=\'\'>-5.90595 -7.11392 0.5 0 -0 0</pose>\n</model>\n')

def add_spheres(f_out,counter,x,y,z,s,k):
	f_out.write('<model name=\'sphere{}\'>\n'.format(counter))
	f_out.write('<pose frame=\'\'>{} {} {} 0 -0 0</pose>\n'.format(x, y, z))
	f_out.write('<scale>{} {} 0.03</scale>\n'.format(s, k))
	f_out.write('<link name=\'link\'>\n')
	f_out.write('<pose frame=\'\'>{} {} {} -0 0 0</pose>\n'.format(x, y, z))
	f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')


def generate_spheres(root_path):
	sphere_dict = {}
	f_out = copy_empty_world(root_path)
	f_out.write('<state world_name=\'default\'>\n<sim_time>1911 852000000</sim_time>\n<real_time>28 221315895</real_time>\n<wall_time>1630639190 308012151</wall_time>\n<iterations>27881</iterations>\n')
	f_out.write('<model name=\'ground_plane\'>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<scale>1 1 1</scale>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<velocity>0 0 0 0 -0 0</velocity>\n')
	f_out.write('<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')
	count = 1
	while(count< 10):
		x = np.random.uniform(-10, 0)
		y = np.random.uniform(-0.5, 0.5)
		z = np.random.uniform(0.5, 1.5)
		add_spheres(f_out, count, x, y, z, 1, 1)
		sphere_dict[(x,y,z)]='sphere{}'.format(count)
		count = count + 1
	f_out.write('</state>\n')
	add_sphere_description(f_out,count)
	f_out.write('</world>\n</sdf>')
	f_out.close()
	return sphere_dict

if __name__ == "__main__":
	root_path = "/root/catkin_ws/src/cps_challenge_2020"
	sphere_dict = generate_spheres(root_path)
	print(sphere_dict)
	with open(root_path+'/worlds/sphere_dict.txt', 'w') as f:
		for key, value in sphere_dict.items():
			f.write('%s:%s\n' % (key, value))
	#with open(root_path+'/worlds/sphere_dict.txt', 'w') as convert_file:
	#	convert_file.write(json.dumps(sphere_dict))