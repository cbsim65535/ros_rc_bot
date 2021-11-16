const Express = require('express')
const Pug = require('pug')
const Stylus = require('stylus')
const Path = require('path')
const BodyParser = require('body-parser');
const Nib = require('nib')

app = Express()
app.set('view engine', 'pug')
app.set('views', Path.join(__dirname, "../pug"))
app.use(BodyParser.urlencoded({
    limit: "10mb",
    parameterLimit: 1000000,
    extended: true
}));
app.use(BodyParser.json({
    limit: '10mb'
}));
app.use(Stylus.middleware({
    src: Path.join(__dirname, '../stylus'),
    dest: Path.join(__dirname, '../www/css'),
    compile: function (str, path) {
        return Stylus(str)
            .set('filename', path)
            .use(Nib())
    }
}))
app.use(Express.static(Path.join(__dirname, "../www")))


app.get("/*", (req, res) => {
    res.render(req.params[0])
})

app.listen(8080)