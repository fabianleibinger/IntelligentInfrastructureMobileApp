import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

void main() {
  test('tests the highlighting of a tile. This tile should be selected', () {
    var tile = generateTile(null, 'drawer', 'drawer', 'Title', Icons.ac_unit);
    expect(tile.selected, true);
  });

  test('tests the highlighting of a tile. This tile should notbe selected', () {
    var tile =
        generateTile(null, 'currentDrawer', 'drawer', 'Title', Icons.ac_unit);
    expect(tile.selected, false);
  });
}
