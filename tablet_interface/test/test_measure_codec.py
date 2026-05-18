from __future__ import annotations

from pathlib import Path

from sensor_msgs.msg import CompressedImage

from tablet_interface.measure_codec import (
    decode_image_data_url,
    encode_compressed_image_data_url,
    is_legacy_fake_measure_vectors,
    load_demo_measure_image_data_url,
)


def test_decode_image_data_url_validates_prefix_and_payload() -> None:
    assert decode_image_data_url("oops") is None
    assert decode_image_data_url("data:image/png;utf8,abc") is None
    assert decode_image_data_url("data:image/png;base64,@@@@") is None
    assert decode_image_data_url("data:image/png;base64,") is None
    assert decode_image_data_url("data:image/png;base64,AA==") == ("png", b"\x00")


def test_encode_compressed_image_data_url_normalizes_formats() -> None:
    empty = CompressedImage()
    assert encode_compressed_image_data_url(empty) == ""

    msg = CompressedImage()
    msg.format = "image/jpg"
    msg.data = b"\x00"

    assert encode_compressed_image_data_url(msg) == "data:image/jpeg;base64,AA=="


def test_load_demo_measure_image_data_url_reads_expected_asset_layout(
    tmp_path: Path,
) -> None:
    (tmp_path / "extender_ui" / "src" / "assets").mkdir(parents=True)
    image_path = tmp_path / "extender_ui" / "src" / "assets" / "image_measures.png"
    image_path.write_bytes(b"\x89PNG")
    current_file = tmp_path / "src" / "input_interfaces" / "measure_codec.py"
    current_file.parent.mkdir(parents=True)
    current_file.write_text("# placeholder\n")

    assert load_demo_measure_image_data_url(str(current_file)) == "data:image/png;base64,iVBORw=="


def test_load_demo_measure_image_data_url_returns_none_when_asset_is_missing(
    tmp_path: Path,
) -> None:
    current_file = tmp_path / "src" / "input_interfaces" / "measure_codec.py"
    current_file.parent.mkdir(parents=True)
    current_file.write_text("# placeholder\n")

    assert load_demo_measure_image_data_url(str(current_file)) is None


def test_is_legacy_fake_measure_vectors_detects_legacy_payloads() -> None:
    assert is_legacy_fake_measure_vectors(None) is False
    assert is_legacy_fake_measure_vectors("not json") is False
    assert is_legacy_fake_measure_vectors('{"source":"opencv"}') is False
    assert is_legacy_fake_measure_vectors('{"source":"fake_opencv_demo"}') is True
