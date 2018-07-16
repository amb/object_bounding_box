bl_info = {
    "name": "Object Bounding Box",
    "author": "Patrick R. Moore, ambi",
    "version": (0, 1, 1),
    "blender": (2, 7, 3),
    "location": "Operator search menu",
    "description": "Rotates an object to a minimum axis aligned bounding box",
    "warning": "",
    "wiki_url": "",
    "category": "Object"}

import bpy
import bmesh
import math
import random
import time
from mathutils import Vector, Matrix
from bpy.props import BoolProperty, FloatProperty, IntProperty, EnumProperty
import numpy as np

def bbox_orient(bme_verts, mx):
    ''' takes a lsit of BMverts ora  list of vectors '''
    if hasattr(bme_verts[0], 'co'):
        verts = [mx * v.co for v in bme_verts]
    else:
        verts = [mx * v for v in bme_verts]
        
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

def bbox_vol(box):
    V = (box[1]-box[0]) * (box[3]-box[2]) * (box[5]-box[4])
    return V

def main(context, rand_sample, spin_res):
    start = time.time()
    #rand_sample = 400  #randomly select this many directions on a solid hemisphere to measure from
    #spin_res = 180   #180 steps is 0.5 degrees
    
    world_mx = context.object.matrix_world
    scale = world_mx.to_scale()
    trans = world_mx.to_translation()
    
    tr_mx = Matrix.Identity(4)
    sc_mx = Matrix.Identity(4)
    
    tr_mx[0][3], tr_mx[1][3], tr_mx[2][3] = trans[0], trans[1], trans[2]
    sc_mx[0][0], sc_mx[1][1], sc_mx[2][2] = scale[0], scale[1], scale[2]
    r_mx = world_mx.to_quaternion().to_matrix().to_4x4()
    
    me = context.object.data

    bme = bmesh.new()
    bme.from_mesh(me)
    
    convex_hull  = bmesh.ops.convex_hull(bme, input = bme.verts, use_existing_faces = True)
    total_hull = convex_hull['geom']
    
    hull_verts = [item for item in total_hull if hasattr(item, 'co')]
    hull_faces = [item for item in total_hull if hasattr(item, 'no')]
    
    def _sample(iu, iv):
        theta = math.pi * iu  
        phi =  math.acos(2 * iv - 1)
        
        x = math.cos(theta) * math.sin(phi)
        y = math.sin(theta) * math.sin(phi)  
        z = math.cos(phi)
        
        axis = Vector((x,y,z))
        axes.append(axis)

        imin_v = None
        for n in range(0, spin_res):
            angle = math.pi/2 * float(n)/spin_res
            rot_mx = Matrix.Rotation(angle, 4, axis)
            
            box = bbox_orient(hull_verts, rot_mx)
            test_V = bbox_vol(box)
            
            if test_V < imin_v or imin_v == None:
                imin_v = test_V
                min_mx = rot_mx

        return imin_v, min_mx

    def _grid_search(minx, maxx, 
                     miny, maxy, 
                     spread, depth):
        xstep = (maxx - minx)/spread
        ystep = (maxy - miny)/spread

        imin_v, imin_mx = _sample(minx, miny)

        next_x, next_y = 0, 0
        for y in range(spread):
            for x in range(spread):
                sx = xstep/2 + x * xstep
                sy = ystep/2 + y * ystep
                mv, mx = _sample(sx, sy)
                if mv < imin_v:
                    imin_v = mv
                    imin_mx = mx
                    next_x = x
                    next_y = y

        if depth > 0:
            return _grid_search(next_x * xstep, next_x * xstep + xstep, next_y * ystep, next_y * ystep + ystep, spread, depth - 1)
        else:
            return imin_v, imin_mx


    min_mx = Matrix.Identity(4)
    min_box = bbox_orient(hull_verts, min_mx)
    min_V = bbox_vol(min_box)
    print('initial volume %f' % min_V)
    min_axis = Vector((0,0,1))
    min_angle = 0
    axes = []

    min_V, min_mx = _grid_search(0.0, 1.0, 0.0, 1.0, 4, 4)

    # for i in range(0,rand_sample):
    #     u = random.random()
    #     v = random.random()

    #     mv, mx = _sample(u, v)

    #     if mv < min_V:
    #         min_V = mv
    #         min_mx = mx
        
        # theta = math.pi * u  
        # phi =  math.acos(2 * v - 1)
        
        # x = math.cos(theta) * math.sin(phi)
        # y = math.sin(theta) * math.sin(phi)  
        # z = math.cos(phi)
        
        # axis = Vector((x,y,z))
        # axes.append(axis)
        # for n in range(0, spin_res):
        #     angle = math.pi/2 * float(n)/spin_res
        #     rot_mx = Matrix.Rotation(angle, 4, axis)
            
        #     box = bbox_orient(hull_verts, rot_mx)
        #     test_V = bbox_vol(box)
            
        #     if test_V < min_V:
        #         min_V = test_V
        #         min_axis = axis
        #         min_angle = angle
        #         min_box = box
        #         min_mx = rot_mx

    elapsed_time = time.time() - start
    print('%f seconds' % (elapsed_time))
    print("final volume %f" % bbox_vol(min_box))     

    bme.free() 

    context.object.matrix_world = tr_mx * r_mx * min_mx * sc_mx
   
    
class ObjectMinBoundBox(bpy.types.Operator):
    """Find approximate minimum bounding box of object"""
    bl_idname = "object.min_bounds"
    bl_label = "Min Bounding Box"

    # generic transform props
    area_sample = IntProperty(
            name="Iterations",
            description = 'number of random directions to test calipers in',
            default = 400)

    angular_sample = IntProperty(
            name="Direction samples",
            description = 'angular step to rotate calipers 90 = 1 degree steps, 180 = 1/2 degree steps',
            default = 180)

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == 'MESH'

    
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


    def draw(self, context):
        layout = self.layout
        
        row =layout.row()
        row.prop(self, "area_sample")
        
        row =layout.row()
        row.prop(self, "angular_sample")
    
    
    def execute(self, context):
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
        main(context, self.area_sample, self.angular_sample)
        return {'FINISHED'}


def register():
    bpy.utils.register_class(ObjectMinBoundBox)


def unregister():
    bpy.utils.unregister_class(ObjectMinBoundBox)


if __name__ == "__main__":
    register()
