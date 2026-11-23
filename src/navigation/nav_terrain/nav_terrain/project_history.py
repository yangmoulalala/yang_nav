"""Bounded undo and redo snapshots for terrain patch edits."""

import numpy as np


class ProjectHistory:
    def __init__(self):
        self.undo_stack, self.redo_stack = [], []
        self.before = None

    def clear(self):
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.before = None

    def begin(self, project):
        if self.before is None:
            self.before = project.owner.copy(), _snapshot(project.ann)

    def commit(self, project):
        owner, annotations = self.before
        self.before = None
        changed = owner != project.owner
        count = np.count_nonzero(changed)
        if not count and annotations == project.ann:
            return 0
        record = None
        if count:
            rows, cols = np.nonzero(changed)
            region = np.s_[rows.min():rows.max() + 1, cols.min():cols.max() + 1]
            record = region, owner[region].copy(), project.owner[region].copy()
        self.undo_stack.append((record, annotations, _snapshot(project.ann)))
        del self.undo_stack[:-200]
        self.redo_stack.clear()
        if count:
            project.refresh()
        return count or 1

    def _apply(self, project, source, destination, use_old):
        if not source:
            return False
        record, annotations_before, annotations_after = source.pop()
        if record is not None:
            region, before, after = record
            project.owner[region] = before if use_old else after
        annotations = annotations_before if use_old else annotations_after
        project.ann = _snapshot(annotations)
        destination.append((record, annotations_before, annotations_after))
        if record is not None:
            project.refresh()
        return True

    def undo(self, project):
        return self._apply(project, self.undo_stack, self.redo_stack, True)

    def redo(self, project):
        return self._apply(project, self.redo_stack, self.undo_stack, False)


def _snapshot(annotations):
    return {key: dict(value) for key, value in annotations.items()}
