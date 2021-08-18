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

TFData = pd.read_csv('C:/Users/johnf/OneDrive/Documents/FogMonitor/Data/Calibration/TF_June21.csv')
DSData = pd.read_csv('C:/Users/johnf/OneDrive/Documents/FogMonitor/Data/Calibration/DS_June21.csv')


new_header = DSData.iloc[0] 
DSData = DSData[1:]
DSData.columns = new_header

TFData["Time"] = pd.to_datetime(TFData["datetime"])
DSData["date time"] = pd.to_datetime(DSData["date time"])

CompData = pd.DataFrame()

CompData["B_Ext"] = ""
CompData["B_Time"] = ""
CompData["B_Humid"] = ""
CompData["TFTime"] = TFData["Time"]
CompData["TFValue"] = TFData["data"]
CompData["TDiff"] = ""

CompData["B_Time"] = pd.to_datetime(CompData["B_Time"])
CompData["TFTime"] = pd.to_datetime(CompData["TFTime"])

def CompTime(b_time, t_time):
    tdiff = abs(b_time - t_time)
    return tdiff

for x in range(0, CompData.shape[0]):
    prevtime = delta
    #print(TFData.iloc[x,2]) 
    for y in range(0, DSData.shape[0]):
        thistime = CompTime(CompData.iloc[x,3], DSData.iloc[y,0])
        if thistime < prevtime:
            prevtime = thistime
            CompData.iloc[x,0] = DSData.iloc[y,8] ##Bring over timestamp
            CompData.iloc[x,1] = DSData.iloc[y,0] ##Bring over bext
            CompData.iloc[x,2] = DSData.iloc[y,1] ##Bring over bext
            CompData.iloc[x,5] = thistime ##save time delta

CompData.to_csv("C:/Users/johnf/OneDrive/Documents/FogMonitor/Data/Calibration/out.csv")