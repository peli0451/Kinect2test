sub replaceLibPath(pFile)
	Set objFS = CreateObject("Scripting.FileSystemObject")
	Set objFile = objFS.OpenTextFile(pFile)
	Set oFile = objFS.CreateTextFile(pFile + "b")
	Do Until objFile.AtEndOfStream
		strLine = objFile.ReadLine
		If InStr(strLine,"<LIBRARY_PROPS_DIR>")> 0 Then
			strLine = vbTab + "<LIBRARY_PROPS_DIR>" + WScript.Arguments(0) + "</LIBRARY_PROPS_DIR>"
		End If 
		oFile.WriteLine(strLine)
	Loop
	objFile.Close()
	oFile.Close()
	objFS.DeleteFile(pFile)
	objFS.MoveFile pFile + "b", pFile
end sub

if WScript.Arguments.Count = 0 then
    WScript.Echo "Expected new library path as parameter!"
end if

replaceLibPath "3DVGR/3DVGR.vcxproj"
replaceLibPath "DirectGL/DirectGL/DirectGL.vcxproj"
replaceLibPath "DirectGL/ShaderCompiler/ShaderCompiler.vcxproj"