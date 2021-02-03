import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:provider/Provider.dart';

final double drawerHeaderHeight = 120;

class AppDrawer extends StatelessWidget {
  AppDrawer(this.currentPage);

  // TODO Make this something dynamic
  final String currentPage;

  @override
  Widget build(BuildContext context) {
    var currentDrawer = Provider.of<DrawerStateInfo>(context).getCurrentDrawer;
    return Drawer(
      child: ListView(
        padding: EdgeInsets.zero,
        children: <Widget>[
          Container(
            height: drawerHeaderHeight,
            child: DrawerHeader(
              decoration: BoxDecoration(
                color: green,
              ),
              child: Text(AppLocalizations.of(context).drawerHeader,
                  style: blackHeader),
            ),
          ),
          generateTile(context, currentDrawer, Routes.main, 'Main',
              Icons.directions_car),
          Divider(),
          generateTile(context, currentDrawer, Routes.vehicle,
              AppLocalizations.of(context).drawerVehicles, Icons.edit),
          generateTile(context, currentDrawer, Routes.settings,
              AppLocalizations.of(context).drawerSettings, Icons.settings),
        ],
      ),
    );
  }
}

//route: Route to page that shall be opened on tap
//text: text shown on tile
//icon: icon shown to the left of the text on the tile
ListTile generateTile(BuildContext context, String currentDrawer, String route,
    String text, IconData icon) {
  return ListTile(
      selected: currentDrawer == route ? true : false,
      leading: Icon(icon),
      title: Text(text),
      onTap: () {
        currentDrawer == route
            ? Navigator.pop(context)
            : Navigator.pushReplacementNamed(context, route);
        //This may be added to the Class itself to update the [DrawerStateInfo] every time the respective widget is built
        Provider.of<DrawerStateInfo>(context, listen: false)
            .setCurrentDrawer(route);
      });
}

// this manages the highlighted item in the drawer.
class DrawerStateInfo with ChangeNotifier {
  String _currentDrawer;
  String get getCurrentDrawer => _currentDrawer;

  void setCurrentDrawer(String drawer) {
    _currentDrawer = drawer;
    notifyListeners();
  }

  void increment() {
    notifyListeners();
  }
}

//TODO implement
List<ListTile> generateVehicles(BuildContext context) {
  List<ListTile> tiles = [];
  tiles.add(
    ListTile(
        leading: Icon(Icons.message),
        title: Text('Messages'),
        onTap: () => Navigator.pushReplacementNamed(context, Routes.vehicle)),
  );
  return tiles;
}
