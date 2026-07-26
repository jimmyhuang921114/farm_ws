"""Efficient, layout-preserving helpers for PointCloud2 diagnostics."""

from typing import Dict, Optional, Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


_DTYPES = {
    PointField.INT8: 'i1',
    PointField.UINT8: 'u1',
    PointField.INT16: 'i2',
    PointField.UINT16: 'u2',
    PointField.INT32: 'i4',
    PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
}


def point_count(message: PointCloud2) -> int:
    """Return the number of complete point records carried by a message."""
    declared = int(message.width) * int(message.height)
    available = len(message.data) // int(message.point_step or 1)
    return min(declared, available)


def field_array(
    message: PointCloud2,
    field_name: str,
    count: Optional[int] = None,
) -> Optional[np.ndarray]:
    """Return a zero-copy one-dimensional view of a scalar point field."""
    field = next(
        (item for item in message.fields if item.name == field_name),
        None,
    )
    if field is None or field.datatype not in _DTYPES or field.count != 1:
        return None

    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(endian + _DTYPES[field.datatype])
    size = point_count(message) if count is None else count
    if size <= 0:
        return np.empty(0, dtype=dtype)

    return np.ndarray(
        shape=(size,),
        dtype=dtype,
        buffer=message.data,
        offset=field.offset,
        strides=(message.point_step,),
    )


def xyz_masks(
    message: PointCloud2,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return finite, all-zero, and valid XYZ masks."""
    count = point_count(message)
    fields = [field_array(message, name, count) for name in ('x', 'y', 'z')]
    if any(field is None for field in fields):
        raise ValueError('PointCloud2 must contain scalar x, y, and z fields')
    x_values, y_values, z_values = fields
    finite = np.isfinite(x_values) & np.isfinite(y_values) & np.isfinite(z_values)
    zero = (x_values == 0.0) & (y_values == 0.0) & (z_values == 0.0)
    return finite, zero, finite & ~zero


def timestamp_statistics(message: PointCloud2) -> Dict[str, object]:
    """Summarize the Livox per-point timestamp field without converting it."""
    values = field_array(message, 'timestamp')
    if values is None:
        return {
            'timestamp_field_present': False,
            'timestamp_nonfinite_count': None,
            'timestamp_min': None,
            'timestamp_max': None,
            'timestamp_range': None,
        }

    finite = np.isfinite(values)
    finite_values = values[finite].astype(np.float64, copy=False)
    if not finite_values.size:
        minimum = maximum = value_range = None
    else:
        minimum = float(np.min(finite_values))
        maximum = float(np.max(finite_values))
        value_range = maximum - minimum

    return {
        'timestamp_field_present': True,
        'timestamp_nonfinite_count': int(values.size - finite_values.size),
        'timestamp_min': minimum,
        'timestamp_max': maximum,
        'timestamp_range': value_range,
    }
