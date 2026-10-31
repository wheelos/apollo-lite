#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Read-only protobuf descriptor adapter; no dependency on fusion execution."""

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory


def require_full_evidence(message):
    publication = getattr(message, "publication", None)
    if publication is not None and publication.detail == 1:
        raise ValueError(
            "FUNCTIONAL_SCENE omits the dense point trace; "
            "use the FULL_EVIDENCE diagnostic channel or cached replay output"
        )


class Reader:
    def __init__(self, schema):
        descriptors = descriptor_pb2.FileDescriptorSet.FromString(schema.read_bytes())
        self.pool = descriptor_pool.DescriptorPool()
        pending = list(descriptors.file)
        loaded = set()
        while pending:
            ready = [
                item for item in pending if all(d in loaded for d in item.dependency)
            ]
            if not ready:
                raise ValueError("unresolved descriptor dependencies")
            for item in ready:
                self.pool.Add(item)
                loaded.add(item.name)
                pending.remove(item)

    def message(self, name):
        return message_factory.GetMessageClass(self.pool.FindMessageTypeByName(name))()

    def read(self, path, name):
        message = self.message(name)
        message.ParseFromString(path.read_bytes())
        return message
