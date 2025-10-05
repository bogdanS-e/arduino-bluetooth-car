#ifndef GET_PERCENT_H
#define GET_PERCENT_H

float getBatteryPercent(float voltage) {
  if (voltage >= 8.20)
    return 100;
  if (voltage >= 8.10)
    return 90 + (voltage - 8.10) / 0.10 * 10;
  if (voltage >= 8.00)
    return 80 + (voltage - 8.00) / 0.10 * 10;
  if (voltage >= 7.90)
    return 70 + (voltage - 7.90) / 0.10 * 10;
  if (voltage >= 7.80)
    return 60 + (voltage - 7.80) / 0.10 * 10;
  if (voltage >= 7.70)
    return 50 + (voltage - 7.70) / 0.10 * 10;
  if (voltage >= 7.60)
    return 40 + (voltage - 7.60) / 0.10 * 10;
  if (voltage >= 7.40)
    return 30 + (voltage - 7.40) / 0.20 * 10;
  if (voltage >= 7.20)
    return 20 + (voltage - 7.20) / 0.20 * 10;
  if (voltage >= 7.00)
    return 10 + (voltage - 7.00) / 0.20 * 10;
  if (voltage >= 6.60)
    return 5 + (voltage - 6.60) / 0.40 * 5;
  if (voltage >= 6.00)
    return (voltage - 6.00) / 0.60 * 5;
  return 0;
}

#endif
