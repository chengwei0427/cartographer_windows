::echo off
for /r .\ %%a in (*.plg;*.opt;*.ncb;*.user;*.suo;*.ilk;*.pdb;*.pch;*.bsc;*.sdf;*.log;*.tlog;*.lastbuildstate;*.obj;*.idb;*.db)do del "%%a"/f /q
::echo .&pause