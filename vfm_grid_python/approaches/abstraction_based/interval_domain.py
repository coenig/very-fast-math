"""Integer interval domain; independent of the legacy queue transition rules.

This module supplies the partition/projection contract, not an SMV compiler.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class IntervalDomain:
    length: int
    gap: int

    def __post_init__(self):
        for name in ('length', 'gap'):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f'{name} must be a positive integer')

    @property
    def intervals(self):
        return ((0, 0), *(
            (lo, min(lo + self.gap - 1, self.length - 1))
            for lo in range(1, self.length, self.gap)
        ), (self.length, self.length))

    def project(self, position):
        if type(position) is not int or not 0 <= position <= self.length:
            raise ValueError('position outside integer road domain')
        if position == 0:
            return 0
        if position == self.length:
            return len(self.intervals) - 1
        return 1 + (position - 1) // self.gap

    def bounds(self, interval):
        if type(interval) is not int or not 0 <= interval < len(self.intervals):
            raise ValueError('invalid interval ID')
        return self.intervals[interval]

    def packing(self, intervals):
        """Return earliest ordered coordinates, or None if no placement fits."""
        positions = []
        for cell in sorted(intervals):
            lo, hi = self.bounds(cell)
            position = max(lo, positions[-1] + self.gap) if positions else lo
            if position > hi:
                return None
            positions.append(position)
        return tuple(positions)
