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
import mathutils as mu
from mathutils import Vector, Matrix
from bpy.props import BoolProperty, FloatProperty, IntProperty, EnumProperty
import numpy as np

import heapq

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

def main(context, rand_sample):
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
    
    def _sample(iu, iv, iz):
        theta = math.pi * iu  
        phi =  math.acos(2 * (iv % 1.0) - 1)
        
        x = math.cos(theta) * math.sin(phi)
        y = math.sin(theta) * math.sin(phi)  
        z = math.cos(phi)
        
        axis = Vector((x,y,z))

        angle = math.pi/2 * iz
        rot_mx = Matrix.Rotation(angle, 4, axis)
        
        box = bbox_orient(hull_verts, rot_mx)
        test_V = bbox_vol(box)

        return test_V, rot_mx


    min_mx = Matrix.Identity(4)
    min_box = bbox_orient(hull_verts, min_mx)
    min_V = bbox_vol(min_box)
    #print('initial volume %f' % min_V)

    best = []
    nudge_step = 0.0
    max_w = 1.0
    N = 0

    for i in range(0,rand_sample):        
        nse = (1.0-nudge_step) ** 2.0
        
        u = (random.random()-0.5) * nse
        v = (random.random()-0.5) * nse
        z = (random.random()-0.5) * nse

        new_pt = mu.Vector((u, v, z))

        # nudge
        if len(best) > 0:
            # cu, cv, cz = 0, 0, 0
            # for t in best:
            #     s = t[0] / max_w
            #     cu += t[2] * s
            #     cv += t[3] * s
            #     cz += t[4] * s

            # center = mu.Vector((cu, cv, cz))
            if len(best) >= 2:
                current  = mu.Vector((best[-1][2], best[-1][3], best[-1][4]))
                # previous = mu.Vector((best[-2][2], best[-2][3], best[-2][4]))
                # delta = current - previous
                # new_pt += current + delta * 0.1 
                new_pt += current
            else:
                new_pt += mu.Vector((best[-1][2], best[-1][3], best[-1][4]))
        else:
            new_pt += mu.Vector((0.5, 0.5, 0.5))
        
        nudge_step += 1.0/rand_sample
        if nudge_step > 1.0:
            nudge_step = 1.0

        # sample
        mv, mx = _sample(*new_pt)
        N += 1

        #print(mv, best[0] if best else [])
        # if best == [] or mv > best[0][0]:
        #     heapq.heappush(best, (1.0/mv, mx, *new_pt))

        #     if len(best) > 20:
        #         heapq.heappop(best)

        #     max_w = sum(t[0] for t in best)

        if mv < min_V:
            min_V = mv
            min_mx = mx

            heapq.heappush(best, (1.0/mv, mx, *new_pt))

            if len(best) > 20:
                heapq.heappop(best)

            max_w = sum(t[0] for t in best)

            if False:
                print("V:{:.2f} B:{:.2f} N:{:.2f}".format(1.0/mv, best[-1][0] if best else 0.0, nudge_step))
                print("uvz:({:.2f} {:.2f} {:.2f})".format(*new_pt))


    elapsed_time = time.time() - start
    if False:
        print('%i iterations in %f seconds' % (N, elapsed_time))
        print('achieved volume %f' % min_V)
        print('score:', elapsed_time * min_V * 1000)

    bme.free() 

    return tr_mx * r_mx * min_mx * sc_mx, min_V
   
    
class ObjectMinBoundBox(bpy.types.Operator):
    """Find approximate minimum bounding box of object"""
    bl_idname = "object.min_bounds"
    bl_label = "Min Bounding Box"

    # generic transform props
    area_sample = IntProperty(
            name="Iterations",
            description = 'number of random directions to test calipers in',
            default = 400)

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == 'MESH'

    
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


    def draw(self, context):
        layout = self.layout
        
        row =layout.row()
        row.prop(self, "area_sample")
    
    
    def execute(self, context):
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
        if True:
            new_mtx, min_vol = main(context, self.area_sample)
            context.object.matrix_world = new_mtx
        else:
            # debug
            avg_vol = 0
            for _ in range(500):
                _, min_vol = main(context, self.area_sample)
                avg_vol += min_vol
            print(avg_vol/500)

        return {'FINISHED'}


def register():
    bpy.utils.register_class(ObjectMinBoundBox)


def unregister():
    bpy.utils.unregister_class(ObjectMinBoundBox)


if __name__ == "__main__":
    register()
