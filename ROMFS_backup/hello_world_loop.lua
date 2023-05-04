-- This script is an example of saying hello

function update()

    gcs:send_text(0, "hello, world")
   
    return update, 10000
    end
   
return update()