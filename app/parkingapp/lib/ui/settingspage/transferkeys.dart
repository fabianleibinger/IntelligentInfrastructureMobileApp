import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/dialogs/scanqrdialog.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
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
          child: createListView(),
        ));
  }

  Widget createListView() {
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
              return Dismissible(
                  background: Container(
                    color: Colors.red,
                  ),
                  key: Key(vehicle.inAppKey),
                  onDismissed: (direction) {
                    //remove vehicle from displayed list
                    vehicleList.remove(vehicle.inAppKey);
                    //remove vehicle from database and bloc
                    DatabaseProvider.db.delete(vehicle.databaseId);
                    BlocProvider.of<VehicleBloc>(context)
                        .add(DeleteVehicle(vehicle));
                    //show snackbar that vehicle has been deleted
                    Scaffold.of(context).showSnackBar(SnackBar(
                      content: Text(vehicle.inAppKey + ' removed!'),
                    ));
                  },
                  child: ListTile(
                    title: Text(vehicle.name),
                    subtitle: Text(vehicle.licensePlate +
                        "; " +
                        vehicle.databaseId.toString()),
                    // TO DO: Implement on Tap
                    onTap: () => Navigator.of(context).push(MaterialPageRoute(
                        builder: (context) => QRPage(
                              vehicle: vehicle,
                            ))),
                  ));
            },
            itemCount: vehicleList.length);
      },
    );
  }
}
