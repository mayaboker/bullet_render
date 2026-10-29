"""World presets must only supply well-formed Bullet positions for known worlds."""
import json

import pytest

from isaac.presets import WORLD_PRESETS, world_preset
from isaac.scenario import vector


def test_unknown_or_missing_world_has_no_preset(tmp_path):
    assert world_preset(None) == {}
    assert world_preset(tmp_path / 'unknown.usd') == {}


def test_presets_are_keyed_by_file_name_and_hold_positions():
    presets = json.loads(WORLD_PRESETS.read_text())
    assert 'rivermark.usd' in presets
    for name, entry in presets.items():
        assert name == name.rsplit('/', 1)[-1]
        assert world_preset('/anywhere/' + name) == entry
        assert set(entry) <= {'target_m', 'spawn_m'}
        for key, value in entry.items():
            assert len(vector(value)) == 3, (name, key)


def test_rivermark_starts_clear_of_the_origin_decoy():
    entry = world_preset('/home/user/ov/rivermark.usd')
    # The origin faces a red post that the detector confuses with the truck; start past it.
    assert entry['spawn_m'][0] >= 10
    assert entry['spawn_m'][2] > 5.7, 'must sit on the lot surface, not at Bullet Z=0'


def test_invalid_preset_entry_is_rejected(tmp_path, monkeypatch):
    import isaac.presets as presets
    bad = tmp_path / 'presets.json'
    bad.write_text(json.dumps({'broken.usd': [1, 2, 3]}))
    monkeypatch.setattr(presets, 'WORLD_PRESETS', bad)
    with pytest.raises(ValueError):
        world_preset('broken.usd')
