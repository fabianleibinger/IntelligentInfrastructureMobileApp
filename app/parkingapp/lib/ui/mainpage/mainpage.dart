import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/userbloc.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/dialogs/parkoutdialog.dart';
import 'package:parkingapp/ui/parkpage/parkpage.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';
import 'package:wifi/wifi.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';

final parkhausImage = 'assets/parkgarage-fasanengarten.jpg';
final parkhausImageHeight = 250;
final bottomMargin = 220;
bool _charge = false;

class MainPage extends StatefulWidget {
  static const String routeName = '/MainPage';
  final String apikey;

  const MainPage({Key key, this.apikey}) : super(key: key);
  @override
  _MainPageState createState() => _MainPageState();
}

class _MainPageState extends State<MainPage> {
  @override
  Widget build(BuildContext context) {
    List<String> _properties = [
      AppLocalizations.of(context).mainPageAvailableSpaces + '79',
      AppLocalizations.of(context).mainPageCarPreferences +
          AppLocalizations.of(context).textNone
    ];
    return Scaffold(
        appBar: AppBar(
          title: Text('Vehicle', style: whiteHeader),
        ),
        drawer: AppDrawer('/MainPage'),
        floatingActionButton: FloatingActionButton.extended(
          onPressed: () {},
          label: Text(AppLocalizations.of(context).actionButtonPark),
        ),
        floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
        body: Column(
          children: [
            Container(
              constraints: BoxConstraints(
                  maxHeight: MediaQuery.of(context).size.height - bottomMargin),
              child: Card(
                margin: EdgeInsets.all(0),
                elevation: 10,
                clipBehavior: Clip.antiAlias,
                child: Column(
                  children: [
                    ListTile(
                      //leading: Icon(Icons.location_on),
                      title: Text(
                          AppLocalizations.of(context).mainPageCarParkTitle),
                      subtitle: Text(AppLocalizations.of(context)
                          .mainPageCarParkSubtitleUndergroundCarPark),
                    ),
                    Container(
                      height: parkhausImageHeight.toDouble(),
                      decoration: BoxDecoration(
                          image: DecorationImage(
                        image: AssetImage(parkhausImage),
                        fit: BoxFit.cover,
                      )),
                    ),
                    Flexible(
                      child: ListView(
                        shrinkWrap: true,
                        children: buildElements(_properties),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ));
  }

  List<Widget> buildElements(List<String> elements) {
    List<Widget> widgets = [];
    widgets.addAll(elements
        .map((element) => ListTile(
              title: Text(element),
            ))
        .toList());
    widgets.add(SwitchListTile(
      title:
          Text(AppLocalizations.of(context).mainPageCarPreferenceShouldCharge),
      onChanged: (bool newValue) {
        setState(() {
          _charge = newValue;
        });
      },
      value: _charge,
    ));
    return widgets;
  }
}
