"""Optional system GEOS acceleration for the same triangle-union area test."""
import ctypes as C
import ctypes.util
import struct
from collections import OrderedDict


class GeosCoverage:
    def __init__(self):
        self.context=None;self.cache=OrderedDict()
        path=C.util.find_library('geos_c')
        if not path:raise OSError('GEOS unavailable')
        self.lib=C.CDLL(path)
        signatures={
            'GEOS_init_r':(C.c_void_p,()),'GEOS_finish_r':(None,(C.c_void_p,)),
            'GEOSGeomFromWKB_buf_r':(C.c_void_p,(C.c_void_p,C.c_char_p,C.c_size_t)),
            'GEOSGeom_createCollection_r':(C.c_void_p,(C.c_void_p,C.c_int,C.POINTER(C.c_void_p),C.c_uint)),
            'GEOSUnaryUnion_r':(C.c_void_p,(C.c_void_p,C.c_void_p)),
            'GEOSPrepare_r':(C.c_void_p,(C.c_void_p,C.c_void_p)),
            'GEOSPreparedCovers_r':(C.c_byte,(C.c_void_p,C.c_void_p,C.c_void_p)),
            'GEOSPreparedGeom_destroy_r':(None,(C.c_void_p,C.c_void_p)),
            'GEOSGeom_destroy_r':(None,(C.c_void_p,C.c_void_p)),
            'GEOSDifference_r':(C.c_void_p,(C.c_void_p,C.c_void_p,C.c_void_p)),
            'GEOSArea_r':(C.c_int,(C.c_void_p,C.c_void_p,C.POINTER(C.c_double)))}
        for name,(result,args) in signatures.items():
            f=getattr(self.lib,name);f.restype=result;f.argtypes=args
        self.context=self.lib.GEOS_init_r()
        if not self.context:raise RuntimeError('GEOS context initialization failed')

    def polygon(self,points):
        points=list(points)+[points[0]]
        wkb=struct.pack('<BIII',1,3,1,len(points))+struct.pack('<'+'dd'*len(points),*(v for p in points for v in p))
        geom=self.lib.GEOSGeomFromWKB_buf_r(self.context,wkb,len(wkb))
        if not geom:raise RuntimeError('GEOS rejected pavement polygon')
        return geom

    def _drop(self,row):
        union,prepared=row
        self.lib.GEOSPreparedGeom_destroy_r(self.context,prepared)
        self.lib.GEOSGeom_destroy_r(self.context,union)

    def uncovered(self,points,indices,triangles):
        key=tuple(indices)
        if not key:return abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1])))/2
        row=self.cache.get(key)
        if row is None:
            geoms=[self.polygon(triangles[i][0]) for i in key]
            array=(C.c_void_p*len(geoms))(*geoms)
            collection=self.lib.GEOSGeom_createCollection_r(self.context,7,array,len(geoms))
            if not collection:
                for geom in geoms:self.lib.GEOSGeom_destroy_r(self.context,geom)
                raise RuntimeError('GEOS collection failed')
            try:union=self.lib.GEOSUnaryUnion_r(self.context,collection)
            finally:self.lib.GEOSGeom_destroy_r(self.context,collection)
            if not union:raise RuntimeError('GEOS pavement union failed')
            prepared=self.lib.GEOSPrepare_r(self.context,union)
            if not prepared:
                self.lib.GEOSGeom_destroy_r(self.context,union);raise RuntimeError('GEOS preparation failed')
            row=(union,prepared);self.cache[key]=row
            if len(self.cache)>512:self._drop(self.cache.popitem(last=False)[1])
        else:self.cache.move_to_end(key)
        body=self.polygon(points)
        try:
            covered=self.lib.GEOSPreparedCovers_r(self.context,row[1],body)
            if covered==1:return 0.
            if covered!=0:raise RuntimeError('GEOS covers failed')
            difference=self.lib.GEOSDifference_r(self.context,body,row[0])
            if not difference:raise RuntimeError('GEOS difference failed')
            try:
                result=C.c_double()
                if not self.lib.GEOSArea_r(self.context,difference,C.byref(result)):raise RuntimeError('GEOS area failed')
                return result.value
            finally:self.lib.GEOSGeom_destroy_r(self.context,difference)
        finally:self.lib.GEOSGeom_destroy_r(self.context,body)

    def __del__(self):
        if self.context:
            for row in self.cache.values():self._drop(row)
            self.lib.GEOS_finish_r(self.context);self.context=None
