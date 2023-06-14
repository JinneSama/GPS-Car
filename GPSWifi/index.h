const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Document</title>

    <style>
        body , h1 , h2 , h3 , h4{
            margin: 0;
            padding: 0;
        }

        form{
            display: flex;
            flex-direction: column;
            gap: 1rem;
        }

        .formContainer{
            position: absolute;
            width: calc(100% - 100px);
            left: 50%;
            transform: translateX(-50%);
        }

        .mainContainer{
            position: relative;
        }
    </style>
</head>
<body>
    <div class="mainContainer">
        <div class="formContainer">
            <form method='POST' action='latlonSave'><h4>Set Longitude and Latitude:</h4>
            <input type='text' placeholder='Latitude' name='lat'/>
            <input type='text' placeholder='Longitude' name='lon'/>
            <input type='submit' value='Send to Device'/></form>
        </div>
    </div>
</body>
</html>
)=====";
