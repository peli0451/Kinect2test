On Error Resume Next

if WScript.Arguments.Count = 0 then
    WScript.Echo "Expected new library path as parameter!"
	WScript.Quit(-1)
end if

Set objFS = CreateObject("Scripting.FileSystemObject")
objFS.DeleteFile("libs.props")
Set objFile = objFS.CreateTextFile("libs.props")
objFile.WriteLine("<?xml version=""1.0"" encoding=""utf-8""?><Project ToolsVersion=""4.0"" xmlns=""http://schemas.microsoft.com/developer/msbuild/2003""><PropertyGroup Label=""UserMacros""><LIBRARY_PROPS_DIR>" + WScript.Arguments(0) + "</LIBRARY_PROPS_DIR></PropertyGroup></Project>")
objFile.Close()