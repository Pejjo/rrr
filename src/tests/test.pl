#!/usr/bin/perl -w

package main;

use rrr::rrr_helper;
use rrr::rrr_helper::rrr_message;
use rrr::rrr_helper::rrr_settings;

sub config {
	my $settings = shift;

	print "perl5 senders: " . $settings->get("senders") . "\n";

	return 1;
}

sub source {
	my $message = shift;

	$message->{'timestamp'} = $message->{'timestamp'};

	$message->send();

	return 1;
}

sub process {
	my $message = shift;

	print "perl5 timestamp: " . $message->{'timestamp'} . "\n";
	print "perl5 old topic: " . $message->{'topic'} . "\n";
	$message->{'topic'} .= "/perl5";
	print "perl5 new topic: " . $message->{'topic'} . "\n";

	$message->send();

	return 1;
}
