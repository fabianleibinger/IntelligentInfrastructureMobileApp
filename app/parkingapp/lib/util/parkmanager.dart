import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/dialogs/noconnectiondialog.dart';
import 'package:parkingapp/notifications/notifications.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/dialogs/parkinggarageoccupieddialog.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

/// Utility class that communicates with the [ApiProvider]
/// to manage park processes for vehicles.
class ParkManager {
  /// Sends the [vehicle] park in request to the [ApiProvider].
  static void parkInRequest(BuildContext context, Vehicle vehicle) {
    if (needsToParkIn(vehicle)) {
      if (currentParkingGarage.vehicleSpecificSpotsAvailable(vehicle)) {
        vehicle.setAndUpdateParkIngIn(context, true);
        print(vehicle.name + ' parking in');
        // Try to contact server.
        ApiProvider.parkIn(vehicle).then((value) {
          /*while (!this.parkedIn) {
            //TODO add functionality
            ApiProvider.getPosition(this).then((value) => null);
          }*/
          //TODO remove
          vehicle.setAndUpdateParkedIn(context, true);
          print('vehicle parked in: ' + vehicle.parkedIn.toString());

          // vehicle not parking in anymore.
        }).whenComplete(() {
          vehicle.setAndUpdateParkIngIn(context, false);
          _checkAndReactParkInWorked(context, vehicle);
        });
      } else {
        // No parking spots available.
        print('no parking spots available');
        showDialog(
            context: context,
            builder: (context) {
              return ParkingGarageOccupiedDialog.getDialog(context);
            });
      }
    } else {
      // vehicle is already parked in.
      print('vehicle ' + vehicle.name + ' is already parked in');
    }
  }

  /// Returns if [vehicle] needs to be parked in.
  static bool needsToParkIn(Vehicle vehicle) {
    return !vehicle.parkedIn && !vehicle.parkingIn;
  }

  /// Checks if park in worked,
  /// creates parked in [Notification] or opens [NoConnectionDialog].
  static void _checkAndReactParkInWorked(
      BuildContext context, Vehicle vehicle) {
    if (vehicle.parkedIn) {
      // if park in worked: notification, that triggers parkOut method.
      Notifications.createNotificationClickable(
          AppLocalizations.of(context).notificationParkInTitle +
              vehicle.name +
              ' ' +
              vehicle.licensePlate,
          AppLocalizations.of(context).notificationParkInBody,
          vehicle.inAppKey, (value) {
        vehicle.parkOut(context);
        return null;
      }, true, false);
    } else {
      // if park in didn't work: connection to server failed.
      showDialog(
          context: context,
          builder: (context) {
            return NoConnectionDialog.getDialog(context);
          });
    }
  }

  /// Sends the [vehicle] park out request to the [ApiProvider].
  static void parkOutRequest(BuildContext context, Vehicle vehicle) {
    if (needsToParkOut(vehicle)) {
      // Cancelling park in if needed.
      vehicle.setAndUpdateParkIngIn(context, false);

      vehicle.setAndUpdateParkIngOut(context, true);
      print(vehicle.name + ' parking out');

      // Try to contact server.
      ApiProvider.parkOut(vehicle).then((value) {
        vehicle.setAndUpdateParkedIn(context, false);
        print('vehicle parked out: ' + vehicle.parkedIn.toString());
        // Open main page.
        Navigator.pushReplacementNamed(context, vehicle.inAppKey);
        showDialog(
            context: context,
            builder: (context) {
              return ParkDialogs.getParkOutFinishedDialog(context);
            });

        // vehicle not parking out anymore.
      }).whenComplete(() {
        vehicle.setAndUpdateParkIngOut(context, false);
        _checkAndReactParkOutWorked(context, vehicle);
      });
    }
  }

  /// Returns if [vehicle] needs to be parked out.
  static bool needsToParkOut(Vehicle vehicle) {
    return !vehicle.parkingOut;
  }

  /// Checks if park out worked,
  /// creates parked out [Notification] or opens [NoConnectionDialog].
  static void _checkAndReactParkOutWorked(
      BuildContext context, Vehicle vehicle) {
    if (!vehicle.parkedIn) {
      // if park out worked: notification
      Notifications.createNotification(
          AppLocalizations.of(context).notificationParkOutTitleOne +
              vehicle.name +
              ' ' +
              vehicle.licensePlate +
              AppLocalizations.of(context).notificationParkOutTitleTwo,
          AppLocalizations.of(context).notificationParkOutBody,
          true,
          false);
    } else {
      // if park out didn't work: connection to server failed
      showDialog(
          context: context,
          builder: (context) {
            return NoConnectionDialog.getDialog(context);
          });
    }
  }
}
