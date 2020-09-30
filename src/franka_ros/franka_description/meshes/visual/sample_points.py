import numpy as np
import open3d as o3d



all_links = ['link0', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'hand', 'kinect']
#for i in all_links:
for i in ['rightfinger', 'leftfinger']:	
	mesh = o3d.io.read_triangle_mesh(i+'.ply')
	cld = mesh.sample_points_poisson_disk(6000)
	#pts = np.asarray(cld.points)
	o3d.io.write_point_cloud(i+'.pcd', cld)
	print('done writing '+ str(i))
	

