import pandas as pd
from datetime import timedelta

delta = timedelta(
     days=50,
     seconds=27,
     microseconds=10,
     milliseconds=29000,
     minutes=5,
     hours=8,
     weeks=2
)

TFData = pd.read_csv('C:/Users/johnf/OneDrive/Documents/FogMonitor/Data/June21.csv')
DSData = pd.read_csv('C:/Users/johnf/OneDrive/Documents/FogMonitor/Data/DS_June2021.csv')

new_header = DSData.iloc[0] 
DSData = DSData[1:]
DSData.columns = new_header

CompData = pd.DataFrame()

CompData["BEXT"] = DSData["bext (1/km)"]
CompData["B_Time"] = DSData["date time"]
CompData["TFTime"] = ""
CompData["TFValue"] = ""
CompData["TDiff"] = ""

CompData["B_Time"] = pd.to_datetime(CompData["B_Time"])
TFData["Time"] = pd.to_datetime(TFData["datetime"])



def CompTime(b_time, t_time):
    tdiff = abs(b_time - t_time)
    return tdiff

for x in range(0, CompData.shape[0]):
    prevtime = delta
    for y in range(0, TFData.shape[0]):
        thistime = CompTime(CompData.iloc[x,1], TFData.iloc[y,2])
        if thistime < prevtime:
            prevtime = thistime
            CompData.iloc[x,2] = TFData.iloc[y,2]
            CompData.iloc[x,3] = TFData.iloc[y,1]
            CompData.iloc[x,4] = thistime