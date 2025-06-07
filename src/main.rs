








fn main() {
    use clap::*;

    let app=Command::new("aa")
        .version("1.0")
        .author("<NAME>. <<EMAIL>>")
        .about("Does awesome things")
        .arg(Arg::new("config")
            .short('c')
            .long("config")
            .value_name("FILE")
            .help("Sets a custom config file")
            )
        .arg(Arg::new("input")
            .index(1)
            )
        .get_matches();

}
