import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';

class AGB extends StatelessWidget {
  static const String routeName = '/agbpage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
          title: Text('AGB und Nutzungsbedingungen', style: whiteHeader),
          leading: IconButton(
            icon: Icon(Icons.arrow_back_ios),
            onPressed: () {
              Navigator.popAndPushNamed(context, SettingsPage.routeName);
            },
          )),
      body: ScrollableAGB(),
    );
  }
}

class ScrollableAGB extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(children: [
      Padding(
        padding: EdgeInsets.all(10),
      ),
      Expanded(
        child: SingleChildScrollView(
          child: Text(
              'Lorem ipsum dolor sit amet, ei quod vide pertinacia sit, et vix error eligendi iracundia. Erat equidem intellegam vis ea, iusto tation doctus ad sea. Eam regione epicuri nominati ad, ne sit possim minimum. Cu quis aliquip pri, id bonorum epicurei vim. Vim denique aliquando reprehendunt at. At vim virtute menandri postulant, scripta facilisi eu cum.Ne vis ferri nemore ponderum. An eos amet tation signiferumque. Dicit dolore ei nam, qui ei feugiat deleniti. Nullam necessitatibus sea ei. Id pro liber vocentEos omnium audire admodum cu. Usu ceteros insolens cu. Vim inani utinam nonumy cu. Eruditi propriae et vix. Te est efficiendi definitionem, eam lucilius salutatus expetendis ex, cum utroque sensibus at. In errem noluisse reprehendunt vel, viderer delicata vel nIllud harum assueverit eu vim, cu sale vitae eam. Eu gloriatur assueverit pri, purto dicit et eam. Pertinax adversarium eu his, eu mel mucius animal, cu mel altera corrumpit. Sit cu dolorum debitis verterem, pri id dicant mollis, platonem forensibus his no. Id veri vulputate repudiandae mei, ei homero euismod vix.Causae consetetur est in, sed ei labitur ocurreret neglegentur. Has porro numquam adversarium ex, quo nostrud legendos ne. No qui laoreet delicatissimi, tale ceteros te pro, per et habeo adolescens intellegam. Est at viderer menandri principes, mei ne nibh mucius feugiat. An solet platonem nec. Partiendo laboramus usu ex'),
        ),
      ),
    ]);
  }
}
