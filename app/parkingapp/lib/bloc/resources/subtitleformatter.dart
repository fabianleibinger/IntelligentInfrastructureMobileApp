import 'package:flutter/cupertino.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/classes/vehicle.dart';

class SubtitleFormatter {
  //Returns a plain text widget with a list of park preferences
  static Text vehicleParkPreferences(
      {@required BuildContext context, @required Vehicle vehicle}) {
    StringBuffer text = StringBuffer();
    if (vehicle.parkingCard) {
      text.write(AppLocalizations.of(context).parkingCard);
      text.write(AppLocalizations.of(context).commaSeperatedList);
    }
    if (vehicle.nearExitPreference) {
      text.write(AppLocalizations.of(context).nearExitPreference);
      text.write(AppLocalizations.of(context).commaSeperatedList);
    }
    if (text.length >= AppLocalizations.of(context).commaSeperatedList.length)
      // remove the last comma and return
      return Text(text.toString().substring(
          0,
          text.length -
              AppLocalizations.of(context).commaSeperatedList.length));
    else
      return null;
  }

  //Returns a plain text wiget with a list of length, width and height
  static Text vehicleDimensions(
      {@required BuildContext context, @required Vehicle vehicle}) {
    return Text(AppLocalizations.of(context).length +
        AppLocalizations.of(context).colonSpace +
        (vehicle.length / 1000).toStringAsPrecision(3) +
        AppLocalizations.of(context).meterShort +
        AppLocalizations.of(context).space +
        AppLocalizations.of(context).width +
        AppLocalizations.of(context).colonSpace +
        (vehicle.width / 1000).toStringAsPrecision(3) +
        AppLocalizations.of(context).meterShort +
        AppLocalizations.of(context).space +
        AppLocalizations.of(context).height +
        AppLocalizations.of(context).colonSpace +
        (vehicle.height / 1000).toStringAsPrecision(3) +
        AppLocalizations.of(context).meterShort);
  }
}
