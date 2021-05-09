namespace Constants
{
  //переделать потом эту константу на пересчет в программе, если меняется выставленный шаг на шаговике
  extern const int extremePositionOfTower = 3500; //ограничение максимального количества шагов (при выставлении 1/4  шага) поворота в одну сторону 1750 * 2
  
  extern const int lagBetweenMoveGun = 50;
  extern const int lenghtOfSerialCommand = 4;
  extern const char* response_TYPE = "900001#";
  extern const char* response_OK = "888888#";
}
