# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pointcloud.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x10pointcloud.proto\x12\x0eproto_messages\"7\n\tPointXYZI\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\t\n\x01i\x18\x04 \x01(\x02\"G\n\nPointCloud\x12\x0e\n\x06header\x18\x01 \x01(\t\x12)\n\x06points\x18\x02 \x03(\x0b\x32\x19.proto_messages.PointXYZIb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'pointcloud_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _POINTXYZI._serialized_start=36
  _POINTXYZI._serialized_end=91
  _POINTCLOUD._serialized_start=93
  _POINTCLOUD._serialized_end=164
# @@protoc_insertion_point(module_scope)
