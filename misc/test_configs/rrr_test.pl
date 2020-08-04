#!/usr/bin/perl -w

package main;

use rrr::rrr_helper;
use rrr::rrr_helper::rrr_message;
use rrr::rrr_helper::rrr_settings;

my $global_settings = undef;

sub config {
	# Get the rrr_settings-object. Has get(key) and set(key,value) methods.
	my $settings = shift;

	# If needed, save the settings object
	$global_settings = $settings;

	# Custom settings from the configuration file must be read to avoid warning messages
	# print "my_custom_setting is: " . $settings->get("my_custom_setting") . "\n";

	# Set a custom setting
	$settings->set("my_new_setting", "5");

	return 1;
}

sub source {
	# Receive a template message
	my $message = shift;

	# Do some modifications
	$message->{'timestamp'} = $message->{'timestamp'} - $global_settings->get("my_custom_setting");

	$message->{'topic'} = "aaa/bbb/ccc";

	print "source: new timestamp of message is: " . $message->{'timestamp'} . "\n";
	print "array ptr: " . $message->{'rrr_array_ptr'} . "\n";

	$message->set_tag_str("my_tag", "my_string");
	$message->set_tag_str("my_tag", "my_string");
	$message->set_tag_str("my_tag", "my_string");
	$message->push_tag_str("my_tag", "my_string 2");
	$message->push_tag_str("my_tag", "my_string 3");
	$message->push_tag_str("my_tag", "my_string 4");

	# Should be 4 now

	print "getting tag\n";
	my @values = $message->get_tag("my_tag");
	print ("done, length: " . ($#values + 1) . " (@values)\n");

	print "getting tag at: " . $message->get_tag_at("my_tag", 1) . "\n";

	my $blob = "aaaaaaaaa";
	$message->push_tag_blob("my_blob", $blob, length $blob);

	my @array = ("4", "3", "2");
	my $bin = pack 'H*', 'ab6501d0e75f12020c14da1545a5';

	$message->push_tag("my_auto_1", "aaa");
	$message->push_tag("my_auto_2", 2222);
	$message->push_tag("my_auto_3", -2222);
	$message->push_tag("my_auto_4", \@array);
	$message->push_tag("my_auto_4", \@array);
	$message->push_tag("my_auto_5_bin", $bin);
	$message->push_tag("my_auto_6", 3.141592);
	$message->push_tag_fixp("my_fixp_1", "16#ad4e65.eeee");
	$message->push_tag_fixp("my_fixp_2", "10#3.141592");
	$message->push_tag_fixp("my_fixp_3", 3.141592);
	$message->push_tag_fixp("my_fixp_4", 6666);
	$message->push_tag_blob("my_blob", $bin, length $bin);
	$message->send();

	# Return 1 for success and 0 for error
	return 1;
}

sub process {
	# Get a message from senders of the perl5 instance
	my $message = shift;

	# Do some modifications to the message
	$message->{'timestamp'} = $message->{'timestamp'} - $global_settings->get("my_custom_setting");

	print "process: new timestamp of message is: " . $message->{'timestamp'} . "\n";

	my $message_text = get_from_tag_or_default($message, "log_message", "no message");
	chomp $message_text;

	print "log prefix: '" . get_from_tag_or_default($message, "log_prefix", "no prefix") . "'\n";
	print "log message: '$message_text'\n";

	my @numbers = (0, 1, 2, 3, 4444444444444444444444444444, -5);

	# Create an array in the message and write some values
	push_tag_str($message, "value_a", "This is the 'a' value");
	push_tag_h($message, "value_number", 12345);
	push_tag_array($message, "value_numbers", \@numbers, "h");
	push_tag_blob($message, "value_blob", "abcd");

	$message->{'ip_so_type'} = "tcp";

	# This can be used to duplicate a message if called multiple times
	# $message->send();

	# Return 1 for success and 0 for error
	return 1;
}

sub set_ip {
	my $message = shift;
	my $ip_addr = shift;
	my $port = shift;
	my $ip_so_type = shift; # tcp or udp

	$message->{'ip_addr'} = sockaddr_in($port, inet_aton($ip_addr));
	$message->{'ip_addr_len'} = bytes::length($message->{'ip_addr'});
	$message->{'ip_so_type'} = $ip_so_type;
}

# Returns all values as array reference
sub get_from_tag {
	my $message = shift;
	my $tag = shift;

	for (my $i = 0; $i < @{$message->{'array_tags'}}; $i++) {
		if (@{$message->{'array_tags'}}[$i] eq $tag) {
			return @{$message->{'array_values'}}[$i];
		}
	}

	my @dummy_array;
	
	return \@dummy_array;
}

# Returns first value only
sub get_from_tag_or_default {
	my $message = shift;
	my $tag = shift;
	my $default = shift;
	
	my $result = get_from_tag($message, $tag);
	
	if (@{$result} == 0) {
		return $default;
	}
	
	return @{$result}[0];
}

sub remove_tag {
	my $message = shift;
	my $tag = shift;

	my @array_tags_new;
	my @array_values_new;
	my @array_types_new;
	
	for (my $i = 0; $i < @{$message->{'array_tags'}}; $i++) {
		if (@{$message->{'array_tags'}}[$i] ne $tag) {
			push @array_values_new, @{$message->{'array_values'}}[$i];
			push @array_tags_new, @{$message->{'array_tags'}}[$i];
			push @array_types_new, @{$message->{'array_types'}}[$i];
		}
	}
	
	$message->{'array_tags'} = \@array_tags_new;
	$message->{'array_values'} = \@array_values_new;
	$message->{'array_types'} = \@array_types_new;
	
	return undef;
}

sub push_tag_blob {
	my $message = shift;
	my $tag = shift;
	my $value = shift;
	push_tag($message, $tag, $value, "blob");
}

sub push_tag_str {
	my $message = shift;
	my $tag = shift;
	my $value = shift;
	push_tag($message, $tag, $value, "str");
}

sub push_tag_h {
	my $message = shift;
	my $tag = shift;
	my $value = shift;
	push_tag($message, $tag, $value, "h");
}

sub push_tag {
	my $message = shift;
	my $tag = shift;
	my $value = shift;
	my $type = shift;

	push @{$message->{'array_values'}}, "$value";
	push @{$message->{'array_tags'}}, $tag;
	push @{$message->{'array_types'}}, $type;
}

sub push_tag_array {
	my $message = shift;
	my $tag = shift;
	my $values_ref = shift;
	my $type = shift;

	push @{$message->{'array_values'}}, $values_ref;
	push @{$message->{'array_tags'}}, $tag;
	push @{$message->{'array_types'}}, $type;
}
