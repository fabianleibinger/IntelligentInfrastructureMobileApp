import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';

class AppDrawer extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Drawer(
      child: ListView(
        padding: EdgeInsets.zero,
        children: <Widget>[
          DrawerHeader(
            decoration: BoxDecoration(
              color: green,
            ),
            child: Text(AppLocalizations.of(context).drawerHeader,
                style: blackHeader),
          ),
          ListTile(
              leading: Icon(Icons.message),
              title: Text('Messages'),
              onTap: () =>
                  Navigator.pushReplacementNamed(context, Routes.main)),
          ListTile(
              leading: Icon(Icons.directions_car),
              title: Text(AppLocalizations.of(context).drawerVehicles),
              onTap: () =>
                  Navigator.pushReplacementNamed(context, Routes.vehicle)),
          ListTile(
            leading: Icon(Icons.settings),
            title: Text(AppLocalizations.of(context).drawerSettings),
            onTap: () =>
                Navigator.pushReplacementNamed(context, Routes.settings),
          ),
        ],
      ),
    );
  }
}
