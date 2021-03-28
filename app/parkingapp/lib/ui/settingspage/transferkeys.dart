import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/dialogs/noconnectiondialog.dart';
import 'package:parkingapp/dialogs/scanqrdialogs.dart';
import 'package:parkingapp/dialogs/scanqrdialogs.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:parkingapp/ui/settingspage/qrpage.dart';

class Transferkeys extends StatelessWidget {
  static const routeName = '/transferkeys';
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text('Daten übertragen', style: whiteHeader),
        ),
        body: Container(
          child: createTransferKeysView(),
        ));
  }

  Widget createTransferKeysView() {
    return BlocBuilder<VehicleBloc, List<Vehicle>>(
      buildWhen: (List<Vehicle> previous, List<Vehicle> current) {
        return true;
      },
      builder: (context, vehicleList) {
        return ListView.separated(
            separatorBuilder: (context, index) => Divider(
                  color: lightgrey,
                  thickness: 1.5,
                  height: 5,
                  indent: 15,
                  endIndent: 15,
                ),
            itemBuilder: (BuildContext context, int index) {
              print("vehicleList: $vehicleList");

              Vehicle vehicle = vehicleList[index];
              return ListTile(
                  title: Text(vehicle.name),
                  subtitle: Text(vehicle.licensePlate +
                      "; " +
                      vehicle.databaseId.toString()),
                  onTap: () {
                    //show QR Code not possible if vehicle is currently parking in or out
                    if (vehicle.parkingIn || vehicle.parkingOut) {
                      ScanQRDialogs.getNoScanQRPossibleDialog(context);
                    }
                    showDialog(
                      context: context,
                      builder: (context) {
                        return ScanQRDialogs.getVehicleQRDialog(
                            context, vehicle);
                      },
                    );
                    /*Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (BuildContext context) =>
                                QRPage(vehicle: vehicle)));*/
                    //ScanQRDialog.createVehicleQRDialog(context);
                  });
            },
            itemCount: vehicleList.length);
      },
    );
  }
}
