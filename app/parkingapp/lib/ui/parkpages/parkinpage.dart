import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:image_size_getter/file_input.dart';
import 'package:image_size_getter/image_size_getter.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/models/classes/parkinggarage.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkInPage extends StatefulWidget {
  static const String routeName = '/parkinpage';

  const ParkInPage({Key key}) : super(key: key);

  @override
  _ParkInPageState createState() => _ParkInPageState();
}

class _ParkInPageState extends State<ParkInPage> {
  //variables needed for the overlay
  final controller = ScrollController();
  OverlayEntry sticky;
  GlobalKey stickyKey = GlobalKey();

  @override
  void initState() {
    //add the vehicle overlay
    if (sticky != null) {
      sticky.remove();
    }
    sticky = OverlayEntry(
      builder: (context) => stickyBuilder(context, vehicle.inAppKey),
    );

    SchedulerBinding.instance.addPostFrameCallback((_) {
      Overlay.of(context).insert(sticky);
    });

    super.initState();
    //wait until build finished to call method
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkIn(context));
  }

  @override
  void dispose() {
    //remove the vehicle overlay
    sticky.remove();
    super.dispose();
  }

  //TODO: setState when parkedIn switches

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkIn),
      //button observes parkedIn value of car
      floatingActionButton: ValueListenableBuilder(
          valueListenable: vehicle.parkedInObserver,
          builder: (BuildContext context, bool, Widget child) {
            return FloatingActionButton.extended(
              //cancel or park out button
              label: vehicle.parkedIn
                  ? Text(AppLocalizations.of(context).actionButtonParkOut)
                  : Text(AppLocalizations.of(context).actionButtonCancelPark),
              backgroundColor: red,
              //park out dialog or cancel dialog
              onPressed: vehicle.parkedIn
                  ? () => ParkDialog.createParkOutDialog(context)
                  : () => ParkDialog.createParkInCancelDialog(context),
            );
          }),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: Column(
        children: [
          ListTile(
            title: Text(currentParkingGarage.name),
            subtitle: Text(currentParkingGarage.type.toShortString()),
          ),
          //add the map
          Expanded(
            child: ListView.builder(
                controller: controller,
                itemCount: 1,
                itemBuilder: (context, index) {
                  return Container(
                    key: stickyKey,
                    child: Image(
                      image:
                          AssetImage("assets/parkgarage-fasanengarten-map.jpg"),
                      fit: BoxFit.fill,
                    ),
                  );
                }),
          ),
        ],
      ),
    );
  }

  Widget stickyBuilder(BuildContext context, String inAppKey) {
    //TODO move somewhere else
    Coordinate bottomLeft =
        Coordinate(lattitude: 49.01388810447, longitude: 8.41950527853);
    Coordinate topRight =
        Coordinate(lattitude: 49.0144759205, longitude: 8.42059599234);
    Coordinate vehiclePosition =
        Coordinate(lattitude: 49.01431771428, longitude: 8.42011294615);

    //assume 0x0 to be the bottom left
    Coordinate _topRightAdjusted = Coordinate(
        longitude: topRight.longitude - bottomLeft.longitude,
        lattitude: topRight.lattitude - bottomLeft.lattitude);
    print(_topRightAdjusted);
    Coordinate _vehiclePositionAdjusted = Coordinate(
        lattitude: vehiclePosition.lattitude - bottomLeft.lattitude,
        longitude: vehiclePosition.longitude - bottomLeft.longitude);
    print(_vehiclePositionAdjusted);

    return AnimatedBuilder(
      animation: controller,
      builder: (_, Widget child) {
        final keyContext = stickyKey.currentContext;
        if (keyContext != null) {
          // widget is visible
          //TODO add position of coordinate
          final box = keyContext.findRenderObject() as RenderBox;
          final pos = box.localToGlobal(Offset.zero);
          //icon (done here for dimensions)
          double _iconSize = 16;
          Container _icon = Container(
              width: _iconSize, height: _iconSize, child: Icon(Icons.circle));
          //scale the icons position
          double iconOffsetHeight =
              (box.size.height / _topRightAdjusted.lattitude) *
                      _vehiclePositionAdjusted.lattitude +
                  (_iconSize / 2);
          double iconOffsetWidth =
              (box.size.width / _topRightAdjusted.longitude) *
                      _vehiclePositionAdjusted.longitude -
                  (_iconSize / 2);
          //position the icon
          return Positioned(
            // pos.dy + box.size.height is the bottom left of the map
            //TODO account for Icon size
            top: pos.dy + box.size.height - iconOffsetHeight,
            left: iconOffsetWidth,
            child: _icon,
          );
        }
        return Container();
      },
    );
  }
}
