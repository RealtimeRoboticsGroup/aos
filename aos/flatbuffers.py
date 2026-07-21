import flatbuffers

from aos.events.event_loop_c import ffi


class FlatbufferDetachedBuffer:
    """A Python mapping of the C++ aos::FlatbufferDetachedBuffer class template.

    This is a fairly loose mapping rather than a direct mapping, because the
    differences in type systems and memory management are central to this class.

    No conversions from C++ are provided here, because those must be specific to
    each instantation of the template.

    Python always owns the memory; wrapping C++-owned memory is not supported.
    """

    def __init__(self, buf):
        self._buf = bytes(buf)

    @property
    def buf(self) -> bytes:
        return self._buf

    @property
    def c_buf(self) -> ffi.CData:
        return ffi.from_buffer(self.buf)

    def c_root(self) -> ffi.CData:
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, self.buf, 0)
        return self.c_buf + n

    def c_to_fbs(self, cls, c_ptr: ffi.CData):
        """Returns a non-object API flatbuffers Python object corresponding to `c_ptr` within `self.buf` (or `ffi.NULL`).

        If you want to construct an object API flatbuffers Python object, use its `InitFromObj` on the result."""
        if c_ptr == ffi.NULL:
            return None
        assert c_ptr >= self.c_buf, 'Pointer is not in our buffer'
        assert c_ptr < self.c_buf + len(
            self.buf), 'Pointer is not in our buffer'
        result = cls()
        result.Init(self.buf, ffi.cast('char *', c_ptr) - self.c_buf)
        return result

    def fbs_to_c(self, fbs: any) -> ffi.CData:
        """Returns a ffi pointer corresponding to `fbs`, which must be `None` or reference `self.buf`."""
        if fbs is None:
            return ffi.NULL
        tab = fbs._tab
        assert ffi.addressof(ffi.from_buffer(tab.Bytes)) == self.c_buf
        assert len(tab.Bytes) == len(self.buf)
        return self.c_buf + tab.Pos
